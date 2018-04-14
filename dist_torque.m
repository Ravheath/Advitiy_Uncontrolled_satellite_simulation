function tau_d=dist_torque(time,state)
%% gravity gradient torque
global J GM total_dist_torque SGP gravity_torque aero_torque%Si 
s = state(1:4);
N=(time); %samp time=0.1s
R=rot(s);
r=SGP(2:4,N);%position in ECI
v=SGP(5:7,N);%velocity in ECI
L=cross(r,v); %angular momentum of centre of mass of satellite expressed in inertial co-ordinates
unit_L=L/norm(L); % unit vector along L
unit_r=r/norm(r);
X=cross(unit_L,unit_r);
X=X/(norm(X));
dcm_OI=[X,-unit_L,-unit_r]'; % rotation matrix from Inertial to orbit
r_body=R*dcm_OI*unit_r;
tau_gravity = 3*GM* cross( r_body, J*r_body) / (norm(r))^3;
gravity_torque=[gravity_torque;tau_gravity'];
%% aerodynamic torque 
global Lx Ly Lz Cd v_COM_to_COP_b rho 
b =  zeros(3,6);
b(:,1) = Lx*[1;0;0];
b(:,2) = Ly*[0;1;0];
b(:,3) = Lz*[0;0;1];
b(:,4) = Lx*[-1;0;0];
b(:,5) = Ly*[0;-1;0];
b(:,6) = Lz*[0;0;-1];
v_body=R*dcm_OI*v;
v_body_unit=v_body/(norm(v_body));
Ap=0;
for i=1:6
     x = dot(b(:,i),v_body_unit);
    if(x>0)
            Ap = Ap+x;
    end
end
F_aero = -0.5 * Cd * rho * Ap * norm(v_body) * v_body;
tau_aero =cross(v_COM_to_COP_b, F_aero);
aero_torque=[aero_torque;tau_aero'];
%% Solar pressure torque
% global P_momentum_flux_from_the_sun Ca_Solar_Drag
% v_Sun_i=Si(2:4,N);
% u=R*dcm_OI*v_Sun_i;
% u = u / norm(u);
% Ap_solar=0;
% for i=1:6
%      x = dot(b(:,i),u);
%     if(x>0)
%             Ap_solar= Ap_solar+x;
%     end
% end
% solar_drag = -Ap_solar * Ca_Solar_Drag * P_momentum_flux_from_the_sun * u ;
% tau_solar=cross(v_COM_to_COP_b, solar_drag);
tau_solar=[0;0;0]; 
%% total disturbance
tau_d=tau_gravity+tau_aero+tau_solar;
total_dist_torque=[total_dist_torque;tau_d'];
