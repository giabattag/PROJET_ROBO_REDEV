function op=SMC_attitude_control(u,I)

global k1_attitude_smc k2_attitude_smc k3_attitude_smc tuning_parameter_smc

tuning_parameter = tuning_parameter_smc;
% I = diag([4.21e-3 3.79e-3 7.79e-3]);

k1=k1_attitude_smc;
k2=k2_attitude_smc;
k3=k3_attitude_smc;

% Ix=I(1,1);
% Iy=I(2,2);
% Iz=I(3,3);

dphi=u(1);
dtht=u(2);
dpsi=u(3);

s1=u(4);
s2=u(5);
s3=u(6);

% ism = [(Iy-Iz)/Ix*dpsi*dtht;
%         (Iz-Ix)/Iy*dpsi*dphi; 
%         (Ix-Iy)/Iz*dphi*dtht];

beta = inv(I);

ism = -beta*cross([dphi; dtht; dpsi],I*[dphi; dtht; dpsi]);

asm = ism + tuning_parameter*[dphi; dtht; dpsi];
K = diag([k1, k2, k3]);

op  = beta \ (-asm - K*sign([s1; s2; s3]));
end

