function [time, y]=rk4(F,h,t1,t2,y0)
%this is runge kutta order-4 solver
%y0 must be a vector
N=((t2-t1)/h);
time=zeros(N,1);
time(1,1)=t1;
M=size(y0);
M=M(1);
y=zeros(N,M);
y(1,:)=y0';
for i=1:N-1
    time(i+1)=t1+h*(i);
    p1=0.2*F(time(i),(y(i,:)')); % please note  that 0.2 is multiplied because actual time step of integration is 0.2 sec 
    p2=0.2*F(time(i)+h/2,(y(i,:)'+p1/2));
    p3=0.2*F(time(i)+h/2,(y(i,:)'+p2/2));
    p4=0.2*F(time(i)+h,(y(i,:)'+p3));
    y1=(y(i,:)'+(1/6)*(p1+2*p2+2*p3+p4))';
    q=y1(1:4);
    y1=[q/norm(q),y1(5:M)]; %normalising the quaternion 
    y(i+1,:)=y1;
    
end
