function [u1, angc] = SMCposition(u,p,v,q,drone_params, DroneID,g)

% position control

xdes   = u(1);
dxdes  = 0;
ddxdes = 0;
ydes   = u(2);
dydes  = 0;
ddydes = 0;
zdes   = u(3);
dzdes  = 0;
ddzdes = 0;
psides = u(4);
x      = p(1);
y      = p(2);
z      = p(3);
R = quat2rotm(q);
v = R*v;
dx     = v(1);
dy     = v(2);
dz     = v(3);
m = drone_params(DroneID).mechanical.m;

Kpz=4;Kdz=4;
Kpx=20;Kdx=30;
Kpy=10;Kdy=10;

ddxc = ddxdes + Kdx*(dxdes - dx) + Kpx*(xdes - x);
ddyc = ddydes + Kdy*(dydes - dy) + Kpy*(ydes - y);
ddzc = ddzdes + Kdz*(dzdes - dz) + Kpz*(zdes - z);

t = [ddxc;ddyc;m*(ddzc+g)];

phic = (1/g)*(ddxc*sin(psides) - ddyc*cos(psides));
thtc = (1/g)*(ddxc*cos(psides) + ddyc*sin(psides));
psic = psides;

%     R        = [cos(psi)*cos(tht)-sin(phi)*sin(psi)*sin(tht) -sin(psi)*cos(phi) cos(psi)*sin(tht)+sin(psi)*sin(phi)*cos(tht);
%           cos(tht)*sin(psi)+cos(psi)*sin(phi)*sin(tht)  cos(phi)*cos(psi) sin(psi)*sin(tht)-cos(psi)*cos(tht)*sin(phi);
%                                     -cos(phi)*sin(tht)           sin(phi)                           cos(phi)*cos(tht)];
u1 = dot(t, R*[0; 0; 1]);

angc = [phic; thtc; psic];

