function deri=statederi(time,state)
global K  J s_W_SAT 
% this function gives the derivative of state
% w_ref is the desired angular velocity of satellite i.e. the angular...
% velocity of orbit frame...
% with respect to ECI expressed in orbit frame frame...
% quaternion is expressed as [ scalar ; vector]
% state = [quaternion ; w_BIB ] 
% s is the error quaternion which transforms from orbit frame to body frame 

w_OIO=[0;-1;0]*(s_W_SAT); %angular velocity of orbit w.r.t. inertial frame expressed in orbit
w_ref=w_OIO;
s=state(1:4);
dw=state(5:7);
s0=s(1);
sv=s(2:4);
R=rot(s); 
w=dw+R*w_ref;
w_ref_dot = [0;0;0];
phi = R*cross(dw,w_ref) + R*w_ref_dot; 
s_vdot=0.5*(s0*dw+cross(sv,dw)); %quaternion kinematics
s0_dot=-0.5*sv'*dw; %quaternion kinematics
tau=total_torque(time,state);  
dw_dot=K*(cross(-w,J*w)+tau-J*phi); 
deri=[s0_dot;s_vdot;dw_dot];




