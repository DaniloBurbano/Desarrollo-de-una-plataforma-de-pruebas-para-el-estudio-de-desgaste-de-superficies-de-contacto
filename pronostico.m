%% RUL (1) >> Evoluci?n de alpha pronostico
%  RUL (2) >> Evoluci?n del tiempo
%  PARAMS (1) >> alpha actual
%  PARAMS (2) >> tiempo actual
%  PARAMS (3) >> tiempo on
%  PARAMS (4) >> tiempo off

function RUL = pronostico(params)
if ~isvector(params)
    error('Input must be a vector')
end
global V m b
global J1 J2 r1 r2 B1 B2 Km  Rm Kb L Ts Tl
% PARAMETROS DE LA PLATAFORMA DE PRUEBAS 
%% MOTOR
B1 = 1.5344E-05;
Km=0.0303;
Kb=0.0303;
J1=2.626E-05;
Tl=8.978E-03;
r1=0.0315;% [m]BLDC motor radius
Rm=6.2;
L=0.00059;
%% RUEDA
J2=0.138; %[Kg*m^2]; Inertia of load (in this case wheel)
B2=0.001187;%[Kg*m^2/s]; Viscous damping coefficient of load (in this case wheel)
r2=0.305;% [m]wheel radius 28inch=aprox 0.35m
Ts=0.165;
%% alpha y m
m=0.017516;
% m=0.0175;
ts=0.5;     % Step time
%************* PARAMETERS  ****************

%************* USER INPUTS ****************
ab=params(3);   % average duration of a high 
bc=params(4);      % average duration of a low
th=ab*ones(1000,1);
tl=bc*ones(1000,1);

Vnampu=12; % MEAN 
std=0; % standard deviation
Vnamp=random('Normal',Vnampu,std,[1001,1]); % Inamp= Random input amplitudes  
Vn=[1:th(1)/ts];
Vn=Vnamp(1)*(Vn./Vn);
flag=1;
%% Construcci?n de Vn
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

ta=0;
tb=ta+ts;

ns=1; %numero de simulaciones
limest=10200;
ti=zeros(limest);
xi=zeros(limest,5);
alphas_RUL=zeros(limest,ns);
tfin=zeros(1,ns);
tfin2=zeros(1,ns);
%% Pron?stico
for j=1:ns
    disp(j);
    first=1;
    b=params(1); 
    x0=[0,0,b,m,0];
    ti=zeros(limest,1);
    ti(1)=0;
    tic
    i=1;
    xacu=x0;
    while xacu(3)>1
            V=Vn(i);
            [t,x]=ode23s(@frictiondrivef2,[ta tb],x0); % produce 25 datos por evaluacion
            ld=size(t);
            ldt=ld(1,1);
            xacu=x(ldt,:);  
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
tseg=ts*(1:limest);
l=tfin*2;
te=tseg(1:l)'+params(2); % devolver evolucion del tiempo
ae=alphas_RUL(1:l,1); % devolver evolucion de alpha
RUL=[ae te];
end
