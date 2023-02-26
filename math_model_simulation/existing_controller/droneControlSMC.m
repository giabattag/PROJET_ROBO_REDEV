function [u2] = droneControlSMC(s, omega, drone_params, DroneID)
%droneControlSMC is the SMC function used to control the drone in
%ErskineSingleDrone.slx

% m = drone_params(DroneID).mechanical.m;
% kp_attitude = drone_params(DroneID).control.attitude.kp;
% kd_attitude = drone_params(DroneID).control.attitude.kd;
I = drone_params(DroneID).mechanical.I;

% attitude control
dphi = omega(1);
dtht = omega(2);
dpsi = omega(3);
Ix = I(1,1);
Iy = I(2,2);
Iz = I(3,3);
s1=s(1);
s2=s(2);
s3=s(3);
tuning_parameter = 100;

ism = [(Iy-Iz)/Ix*dpsi*dtht;
        (Iz-Ix)/Iy*dpsi*dphi;
        (Ix-Iy)/Iz*dphi*dtht];
k1=55;
k2=55;
k3=55;
asm = ism + tuning_parameter*[dphi; dtht; dpsi];
beta = inv(I);

% asm = - beta*cross(omega,I*omega) + tuning_parameter*omega;

K = diag([k1, k2, k3]);

u2  = beta \ (-asm - K*sign([s1; s2; s3]));

%     U = [u1; u2];

% %     Mu =...
% %     [kt      kt       kt       kt;
% %      -l/2*kt   l/2*kt     -l/2*kt    l/2*kt;
% %      l/2*kt    -l/2*kt    -l/2*kt    l/2*kt;
% %      kd     kd      -kd     -kd];
% % 
% %     Mu = [Mu(:,3) Mu(:,1) Mu(:,4) Mu(:,2)];
% kt = drone_params(DroneID).motors.kt_prop;
% kd = drone_params(DroneID).motors.kd_prop;
% Pu = ...
% [kt     kt      kt      kt;
%  -kt/2      kt/2     -kt/2       kt/2;
%  kt/2      -kt/2     -kt/2       kt/2;
%  kd     kd     -kd      -kd];
% 
% %     Pu = [Pu(:,2) Pu(:,4) Pu(:,1) Pu(:,3)];
% 
% Mu = drone_params(DroneID).control.allocation_matrix_u;
% 
% omega_props2 = Pu \ [u1; u2];
% FM = Mu*omega_props2;
% 
% thrust = FM(1);
% moments = FM(2:4);

