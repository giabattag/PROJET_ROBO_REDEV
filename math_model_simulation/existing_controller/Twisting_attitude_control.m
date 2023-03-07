function op=Twisting_attitude_control(u,I)

global k11_attitude_stw k12_attitude_stw k13_attitude_stw tuning_parameter_stw

tuning_parameter = tuning_parameter_stw;
% I = diag([4.21e-3 3.79e-3 7.79e-3]);

k11=k11_attitude_stw;
k12=k12_attitude_stw;
k13=k13_attitude_stw;

% Ix=I(1,1);
% Iy=I(2,2);
% Iz=I(3,3);

dphi=u(1);
dtht=u(2);
dpsi=u(3);

s1=u(4);
s2=u(5);
s3=u(6);
v = [u(7); u(8); u(9)];

% ism = [(Iy-Iz)/Ix*dpsi*dtht;
%         (Iz-Ix)/Iy*dpsi*dphi; 
%         (Ix-Iy)/Iz*dphi*dtht];

beta = inv(I);
ism = -beta*cross([dphi; dtht; dpsi],I*[dphi; dtht; dpsi]);

asm = ism + tuning_parameter*[dphi; dtht; dpsi];
K1 = diag([k11, k12, k13]);


op  = beta \ (-asm - K1*sqrt(abs([s1; s2; s3])).*sign([s1; s2; s3]) + v);
end

