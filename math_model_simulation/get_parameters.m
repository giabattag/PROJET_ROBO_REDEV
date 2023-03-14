function [out] = get_parameters(nDrones)
%setDefaultDroneParams sets default drone parameters for nDrones
%   Values are roughly equivalent to a typical 1kg Quadrotor, such as used
%   in the paper Six D, Briot S, Erskine J, and Chriette A. "Identification
%   of the Propeller Coefficients and Dynamic Parameters of a Hovering
%   Quadrotor From Flight Data". IEEE Robotics and Automation Letters,
%   2020.

%% Parameter Structure
% mechanical properties
drone_params.mechanical.m_arm = 0.067;
drone_params.mechanical.m_body = 0.356;
drone_params.mechanical.m_payload = 0;
drone_params.mechanical.m = drone_params.mechanical.m_body+4*drone_params.mechanical.m_arm;

drone_params.mechanical.Thetau = [0, 0, 0, 0];
drone_params.mechanical.Theta2f = [pi/2, pi/2, 0, 0];
drone_params.mechanical.Theta4f = [pi/2, pi/2, pi/2, pi/2];


drone_params.mechanical.Iu = diag([4.21e-3 3.79e-3 7.79e-3]);
drone_params.mechanical.I2f = [4.12   -1.07   0;
                               -1.07  3.33    0;
                               0      0       5.68]*1e-3;
drone_params.mechanical.I4f = diag([3.53e-3 2.37e-3 3.52e-3]);
drone_params.mechanical.I = drone_params.mechanical.Iu;

drone_params.mechanical.Iarmxx = 0.000033;
drone_params.mechanical.Iarmyy = 0.000068;
drone_params.mechanical.Iarmzz = 0.000076;

drone_params.mechanical.l = 0.24;
% motor and propeller properties
drone_params.motors.omega_max = 1150;
drone_params.motors.omega_min = 400;
drone_params.motors.omega0 = [1;1;1;1]*900;
drone_params.motors.gain = 25;
% drone_params.motors.kt_prop = 1;
drone_params.motors.kt_prop = 3.6e-6;
% drone_params.motors.kd_prop = 5.4e-8;
drone_params.motors.kd_prop = 0.0172e-6;
drone_params.motors.kd_prop_rev = 0.038e-6;
% control parameters
kt = drone_params.motors.kt_prop;
kd = drone_params.motors.kd_prop;
kd_rev = drone_params.motors.kd_prop_rev;
l = drone_params.mechanical.l;
larm = 0.09 %m %l*sind(45+11.9)*0.80;
drone_params.mechanical.THT = 0.20717058;
THT = drone_params.mechanical.THT;
% Mu = ...
%     [kt,    kt,     kt,     kt;
%    -l*kt,   l*kt,   0,      0;
%     0,      0,      l*kt,   -l*kt;
%    -kd,     -kd,    kd,     kd];
%   2       4       1       3
Mu =...
    [kt      kt       kt       kt;
     -l/2*kt   l/2*kt     -l/2*kt    l/2*kt;
     l/2*kt    -l/2*kt    -l/2*kt    l/2*kt;
     kd     kd      -kd     -kd];
 
px = larm*cos(pi/4 + THT) - kd_rev*sin(pi/4 + THT)/kt;
py = larm*sin(pi/4 + THT) + kd_rev*cos(pi/4 + THT)/kt;
pz = l*sqrt(2)/2*sin(THT);

drone_params.motors.px = px;
drone_params.motors.py = py;
drone_params.motors.pz = pz;
M2f = ...
     [0      0       kt       kt;
      px*kt     -px*kt     -l/2*kt    l/2*kt;
      -py*kt    py*kt      -l/2*kt    l/2*kt;
       pz*kt     pz*kt      -kd     -kd];
   
M4f = ...
    [0      0       0       0;
     px*kt     -px*kt     px*kt    -px*kt;
     -py*kt    py*kt      py*kt    -py*kt;
      -pz*kt     -pz*kt      pz*kt     pz*kt];

drone_params.control.allocation_matrix_u = Mu;
drone_params.control.allocation_matrix_2f = M2f;
drone_params.control.allocation_matrix_4f = M4f;
drone_params.control.allocation_matrix = drone_params.control.allocation_matrix_u;
% drone_params.control.allocation_matrix = zeros(4,4,3);
% drone_params.control.allocation_matrix(:,:,1) = drone_params.control.allocation_matrix_u;
% drone_params.control.allocation_matrix(:,:,2) = drone_params.control.allocation_matrix_2f;
% drone_params.control.allocation_matrix(:,:,3) = drone_params.control.allocation_matrix_4f;

Pu = ...
    [kt     kt      kt      kt;
     -kt/2      kt/2     -kt/2       kt/2;
     kt/2      -kt/2     -kt/2       kt/2;
     kd     kd     -kd      -kd];
% P2f = ...
%     [kt         0           kt         0;
%      -kt/2      px*kt/l     kt/2       -px*kt/l;
%      -kt/2      -py*kt/l    kt/2       py*kt/l;
%      -kd        pz     -kd     pz];
P2f = ...
     [0      0       kt       kt;
      px*kt/l     -px*kt/l     -kt/2    kt/2;
      -py*kt/l    py*kt/l      -kt/2    kt/2;
       pz*kt     pz*kt      -kd     -kd];

P4f = ...
    [0      0       0       0;
     px*kt/l     -px*kt/l     px*kt/l    -px*kt/l;
     -py*kt/l    py*kt/l     py*kt/l    -py*kt/l;
      -pz*kt     -pz*kt      pz*kt     pz*kt];

drone_params.control.ss_matrix_u = Pu;
drone_params.control.ss_matrix_2f = P2f;
drone_params.control.ss_matrix_4f = P4f;

drone_params.control.state_enum.unfolded = 0; 
drone_params.control.state_enum.folded2 = 1;
drone_params.control.state_enum.folded4 = 2; 

drone_params.control.state = drone_params.control.state_enum.unfolded;

drone_params.control.position.kp = 2;
drone_params.control.position.kd = 1.5;
drone_params.control.velocity.kp = 2;
drone_params.control.attitude.kp = 2;
drone_params.control.attitude.kd = 0.2;
drone_params.control.attitude_rate.kp = 0.5;
drone_params.control.limits.thrust.max = 7.8;
% 0.85*4*kt * drone_params.motors.omega_max^2;
drone_params.control.limits.thrust.min = -3.4;
% 0.25* drone_params.mechanical.m *10;
% sensor parameters
drone_params.sensor.position_noise = 0.05;
drone_params.sensor.velocity_noise = 0.1;
drone_params.sensor.attitude_noise = 0.05;
drone_params.sensor.gyroscope_noise = 0.1;

%transformations
%CREATE VARIABLES TO PARAMETRISE THIS TO A CUSTOM FRAME
%IMPORT MY NONLINEAR ANGLE CODE TO DO IT
drone_params.transforms.body_hinge_p = 1e-2*[  -4.5, -7.1, -0.2; %Hinge 1
                                              4.5, -7.1, -0.2; %Hinge 2
                                              4.5,  7.1, -0.2; %Hinge 3
                                             -4.5,  7.1, -0.2;].'; %hinge 4 %m
                                    
drone_params.transforms.hinge_motor_p = [larm 0 0].'; %m   %We consider a simplification that the thrust wont have a bias in Z axis to make it easier to understand the behavior
%we also consider that Z axis of the Hinge respect DH convention the
%positive theta Z is in folding direction and the X configuration is 0


drone_params.transforms.body_hinge_r(:,:,1) = Rzmat((-pi+0.577)); %([-pi/2,  0, (-pi+0.577); %Hinge 1
drone_params.transforms.body_hinge_r(:,:,2) = Rzmat((-0.577));    %  -pi/2,  0,  -0.577; %Hinge 2
drone_params.transforms.body_hinge_r(:,:,3) = Rzmat((0.577));     %  -pi/2,  0,  0.577; %Hinge 3
drone_params.transforms.body_hinge_r(:,:,4) = Rzmat((pi-0.577));  %  -pi/2,  0,  pi-0.577;]); %hinge 4 %rad
                                               

drone_params.mechanical.Theta = [0, 0, 0, 0];
% syms tht1 tht2 tht3 tht4
% tht_symbol = [tht1 tht2 tht3 tht4];
% for i=1:4
%     drone_params.transforms.motors_to_hing_r(:,:,i) = Rymat(tht_symbol(i));
% end
% drone_params.transforms.hinge_motor_r = ([ pi/2,  0, drone_params.mechanical.Theta(1);
%                                            pi/2,  0, drone_params.mechanical.Theta(2);
%                                            pi/2,  0, drone_params.mechanical.Theta(3);
%                                            pi/2,  0, drone_params.mechanical.Theta(4)]); %rad
                                       

% syms th1 th2 th3 th4
% 
% % modified DH parameter table
% % thetaj = sym([th1,th2,th3,th4]);
% % dj = [0,L1,L2,L3];
% % rj = sym([0,0,0,0]);
% % alphaj = sym([0,0,0,0]);
% 
% % transformation matrix
% % TbTp1 = DHSym(alphaj,dj,thetaj,rj);
% d1 = sqrt(drone_params.transforms.body_hinge_p(1,1)^2 + drone_params.transforms.body_hinge_p(1,2)^2);
% TbTp1 = DHSym(sym([drone_params.transforms.body_hinge_r(1,1),drone_params.transforms.hinge_motor_r(1,1)]),...
%                 sym([d1, drone_params.transforms.hinge_motor_p(1)]),...
%                 sym([drone_params.transforms.body_hinge_r(1,3) , th1]),...
%                 sym([0, 0]));
%             
% d2 = sqrt(drone_params.transforms.body_hinge_p(2,1)^2 + drone_params.transforms.body_hinge_p(2,2)^2);
% TbTp2 = DHSym([drone_params.transforms.body_hinge_r(2,1),drone_params.transforms.hinge_motor_r(2,1)],...
%                 [d2, drone_params.transforms.hinge_motor_p(1)],...
%                 [drone_params.transforms.body_hinge_r(2,3) , th2],...
%                 [0 0]);
% 
%             
% d3 = sqrt(drone_params.transforms.body_hinge_p(3,1)^2 + drone_params.transforms.body_hinge_p(3,2)^2);
% TbTp3 = DHSym([drone_params.transforms.body_hinge_r(3,1),drone_params.transforms.hinge_motor_r(3,1)],...
%                 [d3, drone_params.transforms.hinge_motor_p(1)],...
%                 [drone_params.transforms.body_hinge_r(3,3) , th3],...
%                 [0 0]);
%             
% d4 = sqrt(drone_params.transforms.body_hinge_p(4,1)^2 + drone_params.transforms.body_hinge_p(4,2)^2);
% TbTp4 = DHSym([drone_params.transforms.body_hinge_r(4,1),drone_params.transforms.hinge_motor_r(4,1)],...
%                 [d4, drone_params.transforms.hinge_motor_p(1)],...
%                 [drone_params.transforms.body_hinge_r(4,3) , th1],...
%                 [0 0]);
% 
%%

out = [];
for i = 1:nDrones
  out = [out, drone_params];
end
end