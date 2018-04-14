function angles=quattoeuler(state)
%this function converts quaternion into euler angles. please note the use atan2 function

q0=state(1);
q1=state(2);
q2=state(3);
q3=state(4);

angles=[ atan2(2*(q0*q1+q2*q3),(1-2*(q1)^2-2*(q2)^2));
         asin(2*(q0*q2-q3*q1));
         atan2(2*(q0*q3+q2*q1),(1-2*(q2)^2-2*(q3)^2))];
