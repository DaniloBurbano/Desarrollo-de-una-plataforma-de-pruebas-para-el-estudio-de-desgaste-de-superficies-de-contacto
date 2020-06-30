% sintonizacion Q
clear all
clc
load plantareal_b=10_m=0.017_Tl=9e-3_prueba8.mat
%load plantareal_b=10_m=0.012_dm=0_prueba8.mat
load datosreales_8.mat
global J1 J2 r1 r2 B1 B2 Km Kb V Rm L Ts Tl
%% RUEDA
J2=0.138; %[Kg*m^2]; Inertia of load (in this case wheel)
B2=0.001187;%[Kg*m^2/s]; Viscous damping coefficient of load (in this case wheel)
r2=0.305;% [m]wheel radius 28inch=aprox 0.35m
Ts=0.165;
maxw2=Cn2(:,2)/8.5;
%maxw2=Cn2(:,2)/8.2;
%maxw2=Cn2(:,2)/10.5;
%% MOTOR
B1 = 1.5344E-05;
Km=0.0303;
Kb=0.0303;
J1=2.626E-05;
Tl=8.978E-03;
%Tl=4.978E-03;
r1=0.0315;
%Vin=12*0.9;
Rm=6.2;
L=0.00059;
maxw1=Cn2(:,1)/85;
%maxw1=Cn2(:,1)/83.5;****
%maxw1=Cn2(:,1)/70;

%% Tama?o del vector
%limreal=size(xisimu,1);
limreal=8800;

%%
z=zeros(limreal,2);
z_=zeros(limreal,2);
z_c=zeros(limreal,2);
xi_c=zeros(limreal,2);
x_s=zeros(limreal,4);
x_o=zeros(limreal,5);
x_ceros=zeros(limreal,2);
%*********** Introduce the data file ******************
ndatos=limreal;
xi=[xireal(1:ndatos,:) xisimu(1:ndatos,3:4)];
%xi=[xisimu(1:ndatos,:) xisimu(1:ndatos,3:4)];
%xi=[v1_ruido v2_ruido xisimu(1:ndatos,3:4)];
%****************************************************************************
%% Ruido en las se?ales
ruido1=wgn(limreal,1,10);
ruido2=wgn(limreal,1,0.5);
v1_ruido=xisimu(1:limreal,1)+ruido1;
v2_ruido=xisimu(1:limreal,2)+ruido2;
%xi=[v1_ruido v2_ruido xisimu(1:ndatos,3:4)];
%%
%0.000004
%0.000002
qe1=1.938595531119244e-05;
qe2=3.242338016463019e-06;
qe=1.938595531119244e-05;
%q2=0.000002;
Q=[ qe1   0   0   0; 
    0   qe2   0   0;
    0   0   3.5e-12   0;
    0   0   0   0];%0.001];%0.000001] ;%Q=cov(W), cov.the process noise W 
R=[0.008 0; 0 0.008];

%% 
ns = 1;
v_motor = zeros(limreal,ns);
v_rueda = zeros(limreal,ns);
instante = 50;
%bk = random('Normal',10,0.01,ns,1);
bk=[10, 9.82, 10.3, 10.4, 10.8];
alphas=zeros(limreal,ns);
tiempo_fin=zeros(2,ns);
t_ekf=zeros(limreal,ns);
sigma=zeros(4,ns);

RVSE1_acu=zeros(ns,1);
RVSE2_acu=zeros(ns,1);
q1_acu=zeros(ns,1);
q2_acu=zeros(ns,1);
%% PSO
n_part = 4; %numero de individuos
d_lim=pi;
G=1;
q1=zeros(n_part,1);
q2=zeros(n_part,1);
for cd=1:n_part
%q1(cd,1) = random('unif',0,4e-4);
q1(cd,1) = random('unif',1e-10,1e-13);
q2(cd,1) = random('unif',0.0000001,0.001);
end
vq1= zeros(n_part,1);
vq2= zeros(n_part,1);
% vq1a= 0.000002*ones(n_part,1);
% vq2a= 0.000002*ones(n_part,1);
vq1a= 2e-13*ones(n_part,1);
vq2a= 2e-13*ones(n_part,1);
J=zeros(n_part,1);
J_ant=1.5*ones(n_part,1);
Pbest=[q1,q1];
Pbest_n=zeros(n_part,2);
iteraciones=15;
Gbw=zeros(iteraciones,1);
Gb=0;
Gbest_q1=q1;
Gb_n=3;
c1=1.3;
c2=2.7;
c3=0.2;
mejores_q=zeros(n_part,iteraciones);
erroresEK=zeros(n_part,iteraciones);
%%
iter=1;
while  iter <= iteraciones
for k =1:n_part %Number of simulations (trials)

    Q=[ qe1   0   0   0; 
        0   qe2   0   0;
        0   0   q1(k)   0;
        0   0   0   0];
    
    p1=0.00002;
    p2=0.00002;
    p3=9e-11;
    p4=0;

    P=[p1   0   0   0;    %set initial error covariance of the estimation process-[P prediction error]
       0   p2   0   0;
       0   0   p3   0;
       0   0   0   p4];%0.000001]
    H = [ 1, 0, 0, 0;
          0, 1, 0, 0];%H is the Jacobian of the sensor transfer functions due to the variables involved
    G = eye(4);       %G is the Jacobian of the plant tranfer functions due to the error.
    %W = 1;           %W is the Jacobian of the sensor transfer functions due to the error.

    %% ALPHA Y M
    b=bk(1); % Obtenido de calibracion (deberia ser tecnico) 
    m=0.0175;
    %%
    x_(1,:)=[0,0,b,m,0];
    x0=x_(1,:); % w1,w2,b,m,I
    t0=0;
    ts=0.5;   % ts step time. It has to be synchronized with real plant step time ******* ver tiempo en el programa Danilo
    tF=t0+ts; % tF set the step time of measure
    eg=[0;0];

        %% ***************** Bucle *********************
        for i =1:limreal;%steps-1          %start at time=2 until step-1
        % the real plant
        z(i,:) = [xi(i,1), xi(i,2)];% DATOS REALES
        % prediction
        V=Vnreal(i);
        [t,x]=ode23s(@frictiondrivef2,[t0 tF],x0); 
        ld=size(t);
        ldt=ld(1,1);
        t_acu=t(ldt,:);
        t_ekf(i,k)=t_acu;
        xacu=x(ldt,:);
         % fill the predict state x_ %add the 
                if(xacu(:,2)<=0)
                    xacu(:,2)=0;
                end
                if(xacu(:,5)<=0)
                    xacu(:,5)=0;
                end
                if(xacu(:,1)<=0)
                    xacu(:,1)=0;
                end
        x_(i,:)=xacu;
        x_o(i,:)=xacu;
        xi_c(i,1)=maxw1(i)*xacu(1);
        xi_c(i,2)=maxw2(i)*xacu(2);
        z_(i,:) = [x_(i,1), x_(i,2)];
        z_c(i,:) = [xi_c(i,1), xi_c(i,2)];
        % compute F with x_ %
        w1 = x_(i,1);
        w2 = x_(i,2);
        alpha_ = x_(i,3);
        m_ = x_(i,4);
        % w1 = xi_c(i,1);%*********************************************
        % w2 = xi_c(i,2);
        % alpha_ = x_(i,3);
        % m_ = x_(i,4);

        Fc =[ -(alpha_*r1^2 + B1)/J1,        (alpha_*r1*r2)/J1,    (r1*r2*w2)/J1 - (r1^2*w1)/J1,   0;
             (alpha_*r1*r2)/J2,          -(alpha_*r2^2 + B2)/J2,     (r1*r2*w1)/J2 - (r2^2*w2)/J2,    0;
          -2*alpha_*m_*r1*(r1*w1 - r2*w2), 2*alpha_*m_*r2*(r1*w1 - r2*w2), -m_*(r1*w1 - r2*w2)^2, -alpha_*(r1*w1 - r2*w2)^2;
                 0,                            0,                            0,                        -100];

        F=expm(Fc*ts); % Discrete    
        %F=Fc;    
        % Prediction of the plant covariance
        P = F*P*F' + G*Q*G';

        % Innovation Covariance
        S = H*P*H'+R;
        %   Kalman's gain
        K = P*H'*inv(S);
        %      
        % State check up and update
        %e=(z(i,:)-z_(i,:))'; % [w1 w2]-[w1_ w2_] %*****ERROR SIN CTES******
        e=(z(i,:)-z_c(i,:))'; % [w1 w2]-[w1_ w2_]%*****ERROR CON CTES******
        es(:,i)=e;% to save the output estimation error
        eg=[eg;e];

        %x_(i,1:4) = x_(i,1:4) + (K * e)';
        x_s(i,1:4) = x_(i,1:4) + (K * e)'; % velocidades sin escalamiento para nuevo X0
        x_(i,1:2)=xi_c(i,:);
        x_(i,1:4) = x_(i,1:4) + (K * e)'; 
        ex_=(xi(i,1:4)-x_(i,1:4))'; % (x_tilda)'
        exs(1:4,i)=ex_;% to save the state estimation error
        % Covariance check up and update
        P = (eye(4)-K*H)*P; 

        % update
        t0=tF;
        tF=tF+ts; % tF set the step time of measure
        % x0=xacum;
        %x_0=x(ldt,:);
        x0=[x_s(i,1:4) xacu(1,5)];

        end
    %%    
        errorv1=(r1*(xi(:,1)-maxw1(1:limreal).*z_(:,1)));
        RMSEv1=sqrt(mean(errorv1.^2));
        errorv2=(r2*(xi(:,2)-maxw2(1:limreal).*z_(:,2))).^2;
        RMSEv2=sqrt(mean(errorv2));
        errora=(xi(1:4395,3)-x_(1:4395,3)).^2;
        RMSEalpha=sqrt(mean(errora));
        J(k,1)=RMSEalpha;
        if(J(k,1)<J_ant(k,1))
           Pbest(k,:)=q1(k); 
        end
       

        RVSE1_acu(k,1)=RMSEv1;
        RVSE2_acu(k,1)=RMSEv2;
        %Acumula datos
        mejores_q(k,iter)=q1(k);
        erroresEK(k,iter)=RMSEv2;
        particula=k
    end
    J_ant=J;
    Gb = min(J);
    fila=find(J==Gb,1);
    Gbw(iter,1)=Gb;
    if(Gb<Gb_n)
         Gb_n=Gb;
         Gbest_q1 = q1(fila);
    end
    
    for local=1:n_part
        vq1(local)=c3*vq1a(local)+ c1*rand()*(Pbest(local,1)-q1(local)) + c2*rand()*(Gbest_q1-q1(local));
        vq1a(local)=vq1(local);
        q1(local,1)=q1(local,1)+vq1(local);
        if q1(local,1) < 0
           q1(local,1) = 0;
        end
        if(vq1(local)<(-1e-5))
            vq1(local)=-5e-6;
        end
        if(vq1(local)>1e-5)
            vq1(local)=5e-6;
        end
    end
    iter=iter+1
end
%%  v1 Reales vs observador
figure(1);clf; 
plot(tisimu(1:limreal),r1*xi(:,1),'b')
hold on;
% plot(r1*maxw1(1:8000).*z_(:,1),'r')
plot(tisimu(1:limreal),r1*maxw1(1:limreal).*z_(:,1),'r')
% plot(r1*(z(:,1)-maxw1(1:8000).*z_(:,1)),'c')
plot(tisimu(1:limreal),errorv1,'g')
hold off
title('error en w1')
ylim([-12 20]);
%xlim([-30 30]);
grid on;
legend('v1 real','v1 observador','error');
xlim([0,50]);
ylim([-6,10]);
%% v2 Reales vs observador
figure(2);clf; 
plot(tisimu(1:limreal),r2*xi(:,2),'b')
hold on;
% plot(r2*maxw2(1:8000).*z_(:,2),'r')
plot(tisimu(1:limreal),r2*maxw2(1:limreal).*z_(:,2),'r')
% plot(r2*z(:,2)-r2*maxw2(1:8000).*z_(:,2),'c')
plot(tisimu(1:limreal),r2*z(:,2)-r2*maxw2(1:limreal).*z_(:,2),'g')
errorv2=(r2*(xi(:,2)-maxw2(1:limreal).*z_(:,2))).^2;
title('error en w2 real y observador')
grid on
legend('v2 real','v2 observador','error');
xlim([0,50]);
ylim([-6,10]);
%% Algoritmo PSO
figure(3);clf;
it=1:iteraciones;
for p=1:n_part
plot(it,mejores_q(p,:),'--','LineWidth',1)
hold on
end
hold off
grid on
ylim([0,4e-4])
xlim([1,30])
%%
%save('mejores_q.mat','mejores_q','iteraciones','n_part','Gbest_q1','Pbest');