function op=model2rotors(u)
global m I g kt kd kd_rev l THT larm P2f M2f
u1=u(1);
u2=[u(2);u(3);u(4)];
phi=u(5);
tht=u(6);
psi=u(7);
w = u(8:10);

FM = M2f*inv(P2f)*[u1; u2];
F = FM(1);
M = FM(2:4);

R = Rzmat(psi)*Rymat(tht)*Rxmat(phi);

a = [0; 0; -g] + R*[0; 0; F]/m;

ddx = a(1);
ddy = a(2);
ddz = a(3);

% ddx=g*(tht*cos(psi)+phi*sin(psi));
% ddy=g*(tht*sin(psi)-phi*cos(psi));
% ddz=-g+u1/m;

% dBangles=I \ u2;
% 
% dp=dBangles(1);
% dq=dBangles(2);
% dr=dBangles(3);

dw = I \ (M - cross(w,I*w));
dp = dw(1);
dq = dw(2);
dr = dw(3);

op=[ddx;ddy;ddz;dp;dq;dr];

end