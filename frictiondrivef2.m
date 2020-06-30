%This function contains the states 
%dx(1)=w_1,
%dx(2)=w_2, 
%dx(3)=alpha,
%dx(4)=m
%dx(5)=I(t)
%This functions is used to simulate the Extended system
%This is necessary to build the "Real data" and for the observer
%***************************************************************
function dx=frictiondrivef2(t,x)  % function dx=ebikesys_in(t,x,tc,Ic)
global V
global alpha  m b
global J1 J2 r1 r2 B1 B2 Km Rm Kb L Ts Tl 
dx=zeros(5,1);

%Im = interp1(tc,Ic,t); %+20;
alpha=x(3); % alpha=-m*x(3)+b; % m y b cte for this case
m=x(4);
Ff=alpha*(r1*x(1)-r2*x(2)); % Contact force, it depends on the perpendicular force, a factor, and the diff of t speeds
dx(5)=(1/L)*(-Rm*x(5)-Kb*x(1)+V); % diff eq electrica del motor dx5 corriente
dx(1)=(1/J1)*((-Ff*r1)-B1*x(1)+Km*x(5)-Tl); % diff eq of motor, Im = interp1(tc,Ic,t)
dx(2)=(1/J2)*((Ff*r2)-B2*x(2)-Ts); % diff eq of wheel
dx(3)=-Ff*(r1*x(1)-r2*x(2))*m;% c=1; alphadot=-m(r_1*w_1-r_2*w_2)^2*alpha
dx(4)=0;%it is assumed constant

end
