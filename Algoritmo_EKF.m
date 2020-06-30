%% Extended Kalman filter to estimate alpha and m (states)

clear all
clc
load datos_simu.mat
load datosreales_8.mat
global J1 J2 r1 r2 B1 B2 Km Kb V Rm L Ts Tl

%% RUEDA
J2=0.138; %[Kg*m^2]; Inertia of load (in this case wheel)
B2=0.001187;%[Kg*m^2/s]; Viscous damping coefficient of load (in this case wheel)
r2=0.305;% [m]wheel radius 28inch=aprox 0.35m
Ts=0.165;
% maxw2=Cn2(:,2)/9;
maxw2=Cn2(:,2)/8.2;
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
%maxw1=Cn2(:,1)/90;
maxw1=Cn2(:,1)/83.5;
%maxw1=Cn2(:,1)/70;

%% Tama?o del vector **********************
%limreal=size(xisimu,1);
limreal=9000;
z=zeros(limreal,2);
z_=zeros(limreal,2);
z_c=zeros(limreal,2);
xi_c=zeros(limreal,2);
x_s=zeros(limreal,4);
x_o=zeros(limreal,5);
x_ceros=zeros(limreal,2);
x_=zeros(limreal,5);
%*********** Introduce the data file ******************
ndatos=limreal;
xi=[xireal(1:ndatos,:) xisimu(1:ndatos,3:4)];

q1=1.938595531119244e-05;
q2=3.242338016463019e-06;
Q=[ q1   0   0   0; 
    0   q2   0   0;
    0   0   3.314e-12   0;
    0   0   0   0];

R=[0.008 0; 0 0.008];

%% 
ns = 1;
instante = 1000;
instante2 = 6000;
%bk = random('Normal',10,0.01,ns,1);
bk=[10, 9.82, 10.3, 10.4, 10.8];
alphas=zeros(limreal,ns);
tiempo_fin=zeros(1,ns);
t_ekf=zeros(limreal,ns);
sigma=zeros(4,ns);
sigma2=zeros(4,ns);
for k =1:ns %Number of simulations (trials)
disp(k)
first=1;
p1=0.00002;
p2=0.00002;
p3=9e-11;
p4=0;

P=[p1   0   0   0;    
   0   p2   0   0;
   0   0   p3   0;
   0   0   0   p4];
H = [ 1, 0, 0, 0;
      0, 1, 0, 0];%H is the Jacobian of the sensor transfer functions due to the variables involved
G = eye(4);       %G is the Jacobian of the plant tranfer functions due to the error.
%W = 1;           %W is the Jacobian of the sensor transfer functions due to the error.

%% ALPHA Y M
b=bk(k); % Obtenido de calibracion (deberia ser tecnico) 
m=0.0175;
%%
x_(1,:)=[0,0,b,m,0];
x0=x_(1,:); % w1,w2,b,m,I
t0=0;
ts=0.5;   % ts step time. It has to be synchronized with real plant step time ******* ver tiempo en el programa Danilo
tF=t0+ts; % tF set the step time of measure

eg=[0;0];
tic
%***************** Bucle *********************
for i =2:limreal;%steps-1          %start at time=2 until step-1
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
% K = P*H'*inv(S);
K = P*(H'/S);  
% State check up and update
%e=(z(i,:)-z_(i,:))'; % [w1 w2]-[w1_ w2_] %*****ERROR SIN CTES******
e=(z(i,:)-z_c(i,:))'; % [w1 w2]-[w1_ w2_]%*****ERROR CON CTES******
%es(:,i)=e;% to save the output estimation error
%eg=[eg;e];

%x_(i,1:4) = x_(i,1:4) + (K * e)';
x_s(i,1:4) = x_(i,1:4) + (K * e)'; % velocidades sin escalamiento para nuevo X0
x_(i,1:2)=xi_c(i,:);
x_(i,1:4) = x_(i,1:4) + (K * e)'; 

% Covariance check up and update
P = (eye(4)-K*H)*P; 
% update
t0=tF;
tF=tF+ts; % tF set the step time of measure
% x0=xacum;
%x_0=x(ldt,:);
x0=[x_s(i,1:4) xacu(1,5)];
if x_(i,3)<1 && first==1
  tiempo_fin(1,k)=t_acu;
  %tiempo_fin(2,k)=i;
  first=0;
end

if i==instante                  
 sigma(:,k)=(diag(P));
end
if i==instante2                  
 sigma2(:,k)=(diag(P));
end
end
toc

alphas(:,k)=x_(:,3);
end


% figure(1);clf; hold on;
% plot(r1*xi(:,1),'r') %tangential speed;  r1*w1=v1
% hold on
% plot(r2*xi(:,2),'b')%tangential speed;  r2*w2=v2
% %plot(t_fin,r1*x(:,1)-r2*x(:,2),'k') %difference between v1 and v2
% hold off
% grid on
% xlabel('steps')
% ylabel('Tangential speeds [m/s]')
% title('Real plant behavior')

% figure(2);clf; hold on;
% plot(tisimu(1:ndatos),r1*xisimu(1:ndatos,1),'r') %tangential speed;  r1*w1=v1
% hold on
% plot(tisimu(1:ndatos),r2*xisimu(1:ndatos,2),'b')%tangential speed;  r2*w2=v2
% hold off
% grid on
% xlabel('Tiempo[s]')
% ylabel('Velocidad tangencial [m/s]')
% title('Velocidad tangencialn Datos simulaci?n')
% ylim([-0.5 10])
% xlim([0 45])

%% VELOCIDADES TANGENCIALES FILTRO DE KALMAN
% figure(3);clf; hold on;
% grid on
% plot(r1*z_c(:,1),'r') %tangential speed;  r1*w1=v1
% hold on
% plot(r2*z_c(:,2),'b')%tangential speed;  r2*w2=v2
% hold off

% figure(4)
% plot(tisimu(1:limreal),xi(:,3))
% grid on
% xlabel('Tiempo[s]');
% ylabel('\alpha')
% title('\alpha filtro de Kalman')
% ylim([0 10])

%%
% figure(5);clf; 
% plot(r1*maxw1(1:limreal).*xisimu(1:limreal,1),'b')
% hold on;
% plot(r1*z_c(:,1),'r')
% plot(r1*(maxw1(1:limreal).*xisimu(1:limreal,1)-z_c(:,1)),'c')
% hold off
% title('error en w1')
% ylim([-12 12]);
% %xlim([-30 30]);
% grid on;
% legend('v1 simu','v1 observador','error');

%%  v1 Reales vs observador
figure(6);clf; 
plot(tisimu(1:limreal),r1*xi(:,1),'b')
hold on;
% plot(r1*maxw1(1:8000).*z_(:,1),'r')
plot(tisimu(1:limreal),r1*maxw1(1:limreal).*z_(:,1),'r')
% plot(r1*(z(:,1)-maxw1(1:8000).*z_(:,1)),'c')
errorv1=(r1*(xi(:,1)-maxw1(1:limreal).*z_(:,1)));
plot(tisimu(1:limreal),errorv1,'g')
RMSEv1=sqrt(mean(errorv1.^2))
hold off
title('error en w1')
ylim([-12 20]);
grid on;
legend('v1 real','v1 observador','error');
xlim([0,5000]);
ylim([-6,10]);
%%
y1=r1*(xi(:,1));
errorv1=(r1*(xi(:,1)-maxw1(1:limreal).*z_(:,1)));
abse1=abs(errorv1);
era1=abse1./y1;
era1(isnan(era1))=0;
era1(isinf(era1))=0;
erp1=mean(era1)*100

%% v2 Reales vs observador
figure(7);clf; 
plot(tisimu(1:limreal),r2*xi(:,2),'b')
hold on;
% plot(r2*maxw2(1:8000).*z_(:,2),'r')
plot(tisimu(1:limreal),r2*maxw2(1:limreal).*z_(:,2),'r')
% plot(r2*z(:,2)-r2*maxw2(1:8000).*z_(:,2),'c')
plot(tisimu(1:limreal),r2*z(:,2)-r2*maxw2(1:limreal).*z_(:,2),'g')
errorv2=(r2*(xi(:,2)-maxw2(1:limreal).*z_(:,2))).^2;
RMSEv2=sqrt(mean(errorv2))
title('error en w2 real y observador')
grid on
legend('v2 real','v2 observador','error');
xlim([0,5000]);
ylim([-6,10]);
%%

y2=r2*(xi(:,2));
abse2=abs(r2*(xi(:,2)-maxw2(1:limreal).*z_(:,2)));
era2=abse2./y2;
era2(isinf(era2))=0;
era2(isnan(era2))=0;
erp2=mean(era2)*100

figure(8)
D_=1-(x_(:,3)/b);
plot(tisimu,D_,'b');
grid on
title('Desgaste normalizado D(t)')



