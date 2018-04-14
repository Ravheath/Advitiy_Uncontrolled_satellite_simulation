function R=rot(s)
%% this function takes a quaternion and gives the corresponding Rotation matrix
s0=s(1);
s_v=s(2:4);
R= [(1-(2*(s_v(2))^2 + 2*(s_v(3))^2 )), 2*(s_v(1)*s_v(2)-s0*s_v(3)), 2*(s_v(1)*s_v(3)+s0*s_v(2));
         2*(s_v(1)*s_v(2)+s0*s_v(3)), -(2*(s_v(1))^2 + 2*(s_v(3))^2 - 1), 2*(s_v(2)*s_v(3)-s0*s_v(1));
         2*(s_v(1)*s_v(3)-s0*s_v(2)), 2*(s_v(2)*s_v(3)+s0*s_v(1)), -(2*(s_v(1))^2 + 2*(s_v(2))^2 - 1)];