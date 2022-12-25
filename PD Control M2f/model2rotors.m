function op=model2rotors(u)
global m I g kt kd kd_rev l THT larm
u1=u(1);
u2=[u(2);u(3);u(4)];
phi=u(5);
tht=u(6);
psi=u(7);
w = u(8:10);

% 2rotors
px = larm*cos(pi/4 + THT) - kd_rev*sin(pi/4 + THT);
py = larm*sin(pi/4 + THT) + kd_rev*cos(pi/4 + THT);
pz = l*sqrt(2)/2*sin(THT);

M2f = ...
     [kt        0       kt          0;
      -l/2*kt   px*kt   l/2*kt      -px*kt;
      -l/2*kt   -py*kt  l/2*kt      py*kt;
      -kd       -pz     -kd         -pz];
P = ...
    [kt         0           kt         0;
     -kt/2      px*kt/l     kt/2       -px*kt/l;
     -kt/2      -py*kt/l    kt/2       py*kt/l;
     -kd        -pz     -kd     -pz];

% 4rotors
% Mu =...
%     [kt      kt       kt       kt;
%      -l/2*kt   l/2*kt     -l/2*kt    l/2*kt;
%      l/2*kt    -l/2*kt    -l/2*kt    l/2*kt;
%      kd     kd      -kd     -kd];
% 
% Mu = [Mu(:,3) Mu(:,1) Mu(:,4) Mu(:,2)];
% 
% P = ...
%     [kt     kt      kt      kt;
%      0      -kt     0       kt;
%      kt     0       -kt     0;
%      kd     -kd     kd      -kd];

FM = M2f*inv(P)*[u1; u2];
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