% Drone data

m = 0.18;
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];
g=9.8;
kt = 1;
kd = 1e-3;
kd_rev = 5e-3;
tuning_parameter=100;
d=1;
l = d*sqrt(2); larm = d;
THT = 11.9*pi/180;

% 2rotors
px = larm*cos(pi/4 + THT) - kd_rev*sin(pi/4 + THT);
py = larm*sin(pi/4 + THT) + kd_rev*cos(pi/4 + THT);
pz = l*sqrt(2)/2*sin(THT);

M2f = ...
     [kt        0       kt          0;
      -l/2*kt   px*kt   l/2*kt      -px*kt;
      -l/2*kt   -py*kt  l/2*kt      py*kt;
      -kd       -pz     -kd         -pz];
P2f = ...
    [kt         0           kt         0;
     -kt/2      px*kt/l     kt/2       -px*kt/l;
     -kt/2      -py*kt/l    kt/2       py*kt/l;
     -kd        -pz     -kd     -pz];

% 4rotors
Mu =...
    [kt      kt       kt       kt;
     -l/2*kt   l/2*kt     -l/2*kt    l/2*kt;
     l/2*kt    -l/2*kt    -l/2*kt    l/2*kt;
     kd     kd      -kd     -kd];

Mu = [Mu(:,3) Mu(:,1) Mu(:,4) Mu(:,2)];

Pu = ...
    [kt     kt      kt      kt;
     -kt/2      -kt/2     kt/2       kt/2;
     -kt/2      kt/2     kt/2       -kt/2;
     -kd     kd     -kd      kd];