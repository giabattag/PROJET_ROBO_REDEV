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
larm = l*sind(45)*0.80;
drone_params.mechanical.THT = 0.20717058;
THT = drone_params.mechanical.THT;
% drone_params.control.allocation_matrix = ...
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
drone_params.control.allocation_matrix = zeros(4,4,3);
drone_params.control.allocation_matrix(:,:,1) = drone_params.control.allocation_matrix_u;
drone_params.control.allocation_matrix(:,:,2) = drone_params.control.allocation_matrix_2f;
drone_params.control.allocation_matrix(:,:,3) = drone_params.control.allocation_matrix_4f;

drone_params.control.state_enum.unfolded = 0; 
drone_params.control.state_enum.folded2 = 1;
drone_params.control.state_enum.folded4 = 2; 

% drone_params.control.state = drone_params.control.state_enum.unfolded;

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

out = [];
for i = 1:nDrones
  out = [out, drone_params];
end
end