function op=SMC_position_control(u)

m = 0.624;
g = 9.81;

xdes   = u(1);
dxdes  = u(2);
ddxdes = u(3);
ydes   = u(4);
dydes  = u(5);
ddydes = u(6);
zdes   = u(7);
dzdes  = u(8);
ddzdes = u(9);
psides = u(10);
x      = u(11);
dx     = u(14);
y      = u(12);
dy     = u(15);
z      = u(13);
dz     = u(16);

phi    = u(17);
tht    = u(18);
psi    = u(19);

Kpz=4;Kdz=4;
Kpx=20;Kdx=30;
Kpy=10;Kdy=10;
ddxc = ddxdes + Kdx*(dxdes - dx) + Kpx*(xdes - x);
ddyc = ddydes + Kdy*(dydes - dy) + Kpy*(ydes - y);
ddzc = ddzdes + Kdz*(dzdes - dz) + Kpz*(zdes - z);

t = [ddxc;ddyc;m*(ddzc+g)];

% tnorm=t/norm(t);

phic = (1/g)*(ddxc*sin(psides) - ddyc*cos(psides));
thtc = (1/g)*(ddxc*cos(psides) + ddyc*sin(psides));

% sinTheta =...;
% thtc     =...;
% sinPhi   =...;
% phic     =...;
psic     = psides;

R        = [cos(psi)*cos(tht)-sin(phi)*sin(psi)*sin(tht) -sin(psi)*cos(phi) cos(psi)*sin(tht)+sin(psi)*sin(phi)*cos(tht);
      cos(tht)*sin(psi)+cos(psi)*sin(phi)*sin(tht)  cos(phi)*cos(psi) sin(psi)*sin(tht)-cos(psi)*cos(tht)*sin(phi);
                                -cos(phi)*sin(tht)           sin(phi)                           cos(phi)*cos(tht)];
u1 = dot(t, R*[0; 0; 1]);
op=[u1; phic; thtc; psic];
end