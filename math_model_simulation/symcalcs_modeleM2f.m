% Calculs symboliques pour le controle de la config avec 2 rotors (M2f)

%% MODEL

syms kt kd kd_rev larm l THT px py pz

% THT: angle entre bras et diagonale
% larm: longueur bras
% l: distance entre rotors

px_eq = larm*cos(pi/4 + THT) - kd_rev*sin(pi/4 + THT);
py_eq = larm*sin(pi/4 + THT) + kd_rev*cos(pi/4 + THT);
pz_eq = l*sqrt(2)/2*sin(THT);

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

syms u1 u2 u3 u4
u = sym([u1,u2,u3,u4].'); % commande: u = P[W^2]

FM = simplify(M2f*inv(P)*u);
FB = [0; 0; FM(1)];
MB = FM(2:4);

% equations
syms Ixx Iyy Izz Jr Wr m g

I = diag([Ixx Iyy Izz]);

syms x1  x2   x3 x4  x5  x6   x7 x8 x9 x10 x11 x12
%    phi dphi th dth psi dpsi z  dz x  dx  y   dy
x = sym([x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12].');
deta = sym([x2 x4 x6].');
eta = sym([x1 x3 x5].');
P = sym([x9 x11 x7]).';
dP = sym([x10 x12 x8].');

Rot = ...
    [cos(x5)*sin(x3) cos(x5)*sin(x3)*sin(x1)-sin(x5)*cos(x1) cos(x5)*sin(x3)*sin(x1)+sin(x5)*sin(x1);
     sin(x5)*cos(x3) sin(x5)*sin(x3)*sin(x1)+cos(x5)*cos(x1) sin(x5)*sin(x3)*cos(x1)-cos(x5)*cos(x1);
     -sin(x3)        cos(x3)*sin(x1)                         cos(x3)*cos(x1)];

Rotr = ...
    [1      0           -sin(x3);
     0      cos(x1)     sin(x1)*cos(x3);
     0      -sin(x1)    cos(x1)*cos(x3)];

% rotational dynamics
ddeta = I \ (MB - cross(deta,I*deta) - cross(deta,[0; 0; -Jr*Wr]));

% translational dynamics
ddx = [0; 0; g] + Rot*FB/m;

%% CONTROL





