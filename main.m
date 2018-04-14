clear all;
load('SGP_120k.mat');
load('Bi_120k.mat');
load('Si_120k.mat');
R_e = 6378164 + 668000;    % Earth radius + Height of sat
%% Constants
global total_dist_torque  J K  s_W_SAT GM  Lx Ly Lz Cd rho
global v_COM_to_COP_b P_momentum_flux_from_the_sun Ca_Solar_Drag SGP Si Bi gravity_torque aero_torque
SGP=SGP_120k; % position and velocity data in ECI
Si= Si_120k; %  sun vector (from Earth to Sun) data in EC I
Bi=Bi_120k; % Magnetic field data in ECI 
v_COM_to_COP_b = [9.07; 8.91; 2.98]*1e-3; %vector between centre of mass and centre of pressure from AAUSAT
Ca_Solar_Drag = 1.4; 
rho=3.91e-14;%density of atmosphere 
P_momentum_flux_from_the_sun = 4.4e-6; % momentum flux
Cd=2; % coefficient of atmospheric drag
Ixx =  .17007470856;
Iyy =  .17159934710;
Izz =  .15858572070;
Ixy = -.00071033134;
Iyz = -.00240388659;
Ixz = -.00059844292;
%J=diag([0.0017;0.0022;0.0022]); %moment of inertia of AAUSAT along principal axis 
J=0.01*[Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz];%after scaling moment of inertia of pratham  
%Sides of satellite 
Lx=0.1;
Ly=0.1;
Lz=0.1;

GM=(6.673)*5.9742*1e13;    % G*M_earth, SI
s_W_SAT =sqrt(GM/R_e^3); % angular velocity of revolution of satellite in rad/sec
T_ORBIT = 2*pi/s_W_SAT;  % orbital perio
K=inv(J); % to make computation faster
%% Main simulation 
% please note that the T vector stores index which can be bijectively mapped to time
Ti=16279; %starting of eclipse
h=2; % signifying a time step of 0.2 sec
Tf=36957; % end of eclipse
T=Ti:h:Tf; 
N=size(T); 
N=N(2);
gravity_torque=zeros(1,3);
aero_torque=zeros(1,3);
total_dist_torque=zeros(1,3);
%% Initial condition 
dw0=[0;0;0]; 
s0_v=[0; 0; 0]; %vector part of error quaternion at t=0;
s0_0=sqrt(1-(norm(s0_v))^2); %scalar part of error quaternion at t=0;
s0=[s0_0;s0_v]; % quatrnion at t=0;
state0=[s0;dw0];
[time, states]=rk4(@statederi,h,Ti,Tf,state0);
euler=zeros(3,N-1);
for k=1:N-1 
    euler(:,k)=quattoeuler(states(k,1:4));
end
%% plotting error quaternion, angular velocity, euler angles 
set(groot,'defaultLineLineWidth',2)
set(0,'DefaultaxesLineWidth', 2)
set(0,'DefaultaxesFontSize', 16)
set(0,'DefaultTextFontSize', 16) 
set(0,'DefaultaxesFontName', 'arial') 
set(0,'defaultAxesXGrid','on')
set(0,'defaultAxesYGrid','on')
time=(time-1)*0.1;
figure();
plot(time,states(:,2));
hold on;
plot(time,states(:,3));
plot(time,states(:,4));
xlabel('time(sec)');
legend('sv_1','sv_2','sv_3');
title('vector component of error quaternion');
figure(2);
plot(time,states(:,1));
xlabel('time(sec)');
ylabel('s0');
title('scalar component of error quaternion');
figure();
plot(time,states(:,5));
hold on;
plot(time,states(:,6));
plot(time,states(:,7));
xlabel('time(sec)');
legend('dw_1','dw_2','dw_3');
title('angular vel of body w.r.t to orbit in body frame');
ylabel('rad/sec');
figure();
plot(time,euler(1,:));
hold on;
plot(time,euler(2,:));
plot(time,euler(3,:));
xlabel('time(sec)');
legend('roll','pitch','yaw');
ylabel('angle(rad)')
title('euler angle of body w.r.t to orbit');