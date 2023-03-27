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

t = [m*ddxc;m*ddyc;m*(ddzc+g)];
% ad = [ddxc;ddyc;ddzc+g];

phic = (1/g)*(ddxc*sin(psides) - ddyc*cos(psides));
thtc = (1/g)*(ddxc*cos(psides) + ddyc*sin(psides));
psic = psides;

%     R        = [cos(psi)*cos(tht)-sin(phi)*sin(psi)*sin(tht) -sin(psi)*cos(phi) cos(psi)*sin(tht)+sin(psi)*sin(phi)*cos(tht);
%           cos(tht)*sin(psi)+cos(psi)*sin(phi)*sin(tht)  cos(phi)*cos(psi) sin(psi)*sin(tht)-cos(psi)*cos(tht)*sin(phi);
%                                     -cos(phi)*sin(tht)           sin(phi)                           cos(phi)*cos(tht)];
u1 = dot(t, R*[0; 0; 1]);
% u1 = dot(t, [0; 0; 1]);

% f = saturateThrust(ad,drone_params(DroneID).control.limits.thrust.max, m, g);
% u1 = norm(f);
% zd = f/u1;
% u1 = u1 * zd' * R(:,3);

angc = [phic; thtc; psic];
% angc = angleCommand(zdes, psides);
% 
% function thrust_vector = saturateThrust(ad, fmax, m, g)
% %% saturateThrust
% % fmax^2 = m(g^2 + a^2) - 2m^2(g*a)*cos(th+pi/2)
% 
% if norm(m*(ad + [0;0;g])) > fmax
%   % do thrust saturation
%   th = atan2(sqrt(ad(1)^2 + ad(2)^2), ad(3));
%   b = -2*m^2*g*cos(th + pi/2);
%   c = (m*g)^2 - (fmax)^2;
%   amax = (-b + sqrt(b*b - 4*c*m^2))/(2*m^2);
%   factor = amax/norm(ad);
%   thrust_vector = m*(factor*ad + [0;0;g]);
% else
%   thrust_vector = m*(ad + [0;0;g]);
% end
% 
% if thrust_vector(3) < m*g/3
%   % do downward thrust saturation
% end
% end
% 
% function angc = angleCommand(zd,yd)
% % point the quadrotor axis from currently at zc towards desrired zd
% % K frame is flat rotation about z0
% % L frame is rotated about new y_K by theta
% % normalize inputs
% zd = zd/norm(zd);
% 
% kzd = Rzmat(yd)'*zd;
% th_cmd = atan(kzd(1)/kzd(3));
% 
% lzd = Rymat(th_cmd)'*kzd;
% phi_cmd = atan2(-lzd(2), lzd(3));
% 
% angc = [phi_cmd, th_cmd, yd];
% % qcmd = eul2quat(phi_cmd,th_cmd,yd,'zyx');
% end

end