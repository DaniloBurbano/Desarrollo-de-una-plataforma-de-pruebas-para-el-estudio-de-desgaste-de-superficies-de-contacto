
clear all
clc
global I V
global alpha alpha0 m b
global J1 J2 r1 r2 B1 B2 Km  Rm Kb L Ts Tl
% PARAMETROS DE LA PLATAFORMA DE PRUEBAS 
load datosreales_8.mat
limreal = size(xireal,1);
%% MOTOR
% B1 = 1.60954E-05;  % aproximacion por ajuste de curva
B1 = 1.5344E-05;
% Km=0.0341;
% Kb=0.0341;
Km=0.0303;
Kb=0.0303;
% J1=2.665E-05;
J1=2.626E-05;
Tl=8.978E-03;
%Tl=5.551E-03;
r1=0.0315;% [m]BLDC motor radius
% r2=0.305;% [m]wheel radius 28inch=aprox 0.35m
Vin=12;
Rm=6.2;
L=0.00059;
%ct1=; %constante de escalamiento en el motor
%maxw1=Cn2(:,1)/193.4;
%maxw1=Cn2(:,1)/163.3; % velocidad angular maxima en la simu ANTERIOR
%maxw1=Cn2(:,1)/104;%131.38;***********
maxw1=Cn2(:,1)/83.5;%131.38;
%% RUEDA
J2=0.138; %[Kg*m^2]; Inertia of load (in this case wheel)
B2=0.001187;%[Kg*m^2/s]; Viscous damping coefficient of load (in this case wheel)
r2=0.305;% [m]wheel radius 28inch=aprox 0.35m
% Ts=0.065;
Ts=0.165;
%ct2=; %constante de escalamiento en la rueda
%maxw2=Cn2(:,2)/8.98;
%maxw2=Cn2(:,2)/12.12; Constantes pruebas anteriores
% maxw2=Cn2(:,2)/10.5;%8.43;*********
%maxw2=Cn2(:,2)/8.43;
maxw2=Cn2(:,2)/8.2;
%maxw2=ones(3000,1);
%% alpha y m
b=10;
m=0.017516;
% m=0.0175;
ts=0.5;     % Step time
%************* PARAMETERS  ****************
x0=[0,0,b,m,0]; % Initial conditions **** Note x0 could be a line vector also ****
%%
%************* USER INPUTS ****************
ab=5;   % average duration of a high 
bc=15;      % average duration of a low
th=ab*ones(1000,1);
tl=bc*ones(1000,1);

Vnampu=12; % MEAN 
std=0; % standard deviation
Vnamp=random('Normal',Vnampu,std,[1001,1]); % Inamp= Random input amplitudes  
Vn=[1:th(1)/ts];
Vn=Vnamp(1)*(Vn./Vn);
flag=1;

for j=2:680
   if flag==1
    Vn1=[1:tl(j)/ts];
    Vn1=0*(Vn1./Vn1);
    Vn=[Vn Vn1];
    flag=0;
   else
    Vn1=[1:th(j)/ts];
    Vn1=Vnamp(j)*(Vn1./Vn1);   
    Vn=[Vn Vn1];
    flag=1;
   end
end

sizeVn=size(Vn,2);
limsimu=1000;

ta=0;
tb=ta+ts;

ns=1; %numero de simulaciones
limest=10200;
ti=zeros(limest);
ti(1)=0;
xi=zeros(limest,5);
xi(1,:)=x0;
xi_c=zeros(limest,5);
xi_c(1,:)=x0;
bk=[10];
%bk = random('Normal',10,0.01,ns,1);
alphas_RUL=zeros(limest,ns);
tfin=zeros(1,ns);
tfin2=zeros(1,ns);
%%
for j=1:ns
    disp(j);
    first=1;
    b=bk(j);
    %b=10;
    tacum=0; 
    x0=[0,0,b,m,0];
    ti=zeros(limest,1);
    ti(1)=0;
    tic
    %for i =2:limest
    i=1;
    xacu=x0;
    %for i =1:limest
    while xacu(3)>1
            %V=Vnreal(i); *** Para simular con Vn real
            V=Vn(i);
            [t,x]=ode23s(@frictiondrivef2,[ta tb],x0); % produce 25 datos por evaluacion
            ld=size(t);
            ldt=ld(1,1);
            xacu=x(ldt,:);  %+W; % add W to the last value of evaluation of f
            t_acu=t(ldt,:);
            if(xacu(2)<=0)
                xacu(2)=0;
            end
            if(xacu(5)<=0)
                xacu(5)=0;
            end
            if(xacu(1)<=0)
                xacu(1)=0;
            end
            xi(i,:)=xacu;   % fill the real state xi 
            xi_c(i,1)=maxw1(i)*xacu(1); %w1*cte_escalamiento
            xi_c(i,2)=maxw2(i)*xacu(2); %w2*cte_escalamiento
            ti(i)=t(ldt);   % fill the real time ti 
            % update
            ta=tb;
            tb=tb+ts;       % tF set the step time of measure
            x0=xacu;        % new inicial conditions for the segment
            %xacum=[xacum;xi];
            if xacu(3)<1 && first==1
              tfin(1,j)=t_acu
              %tiempo_fin(2,k)=i;
              first=0;
            end
            i=i+1
    end
    toc
    alphas_RUL(:,j)=xi(:,3);
    tfin2(1,j)=t_acu;
    ta=0;
    tb=ta+ts;
end
%%
%x0=xacum(length(xacum),:)';
% Plot the real plant behavior
i=i-1;
close all

%%
tseg=ts*(1:limest);
figure(1)
for k=1:ns
    l=tfin(k)*2;
    plot(tseg(1:l),alphas_RUL(1:l,k))
    hold on
end
hold off
grid on
xlabel('Tiempo[s]')
ylabel('alpha')
xlim([0,5000])
ylim([0,10])
%% Desgaste D
D_=1-(xi(1:i,3)/10);
figure(2)
for k1=1:ns
    l=tfin(k1)*2;
    plot(tseg(1:l),1-(alphas_RUL(1:l,k1)/10));
    hold on
end
hold off
grid on
title('Desgaste normalizado D?(t)')
xlabel('Tiempo[s]')
ylabel('Desgaste')
xlim([0,5000])
ylim([0,1])
