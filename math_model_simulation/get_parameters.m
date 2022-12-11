function [out] = setDefaultDroneParams(nDrones)
%setDefaultDroneParams sets default drone parameters for nDrones
%   Values are roughly equivalent to a typical 1kg Quadrotor, such as used
%   in the paper Six D, Briot S, Erskine J, and Chriette A. "Identification
%   of the Propeller Coefficients and Dynamic Parameters of a Hovering
%   Quadrotor From Flight Data". IEEE Robotics and Automation Letters,
%   2020.

%% Parameter Structure
% mechanical properties
drone_params.mechanical.m = 1;
drone_params.mechanical.I = [3,0,0;
     0,3,0;
     0,0,6]*1e-3;
drone_params.mechanical.l = 0.15;
% motor and propeller properties
drone_params.motors.omega_max = 1150;
drone_params.motors.omega_min = 400;
drone_params.motors.omega0 = [1;1;1;1]*900;
drone_params.motors.gain = 25;
drone_params.motors.kt_prop = 3.6e-6;
drone_params.motors.kd_prop = 5.4e-8;
% control parameters
kt = drone_params.motors.kt_prop;
kd = drone_params.motors.kt_prop;
l = drone_params.mechanical.l;
larm = l*sind(45)*0.80;
drone_params.mechanical.THT = 11.9 * pi/180;
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
 
px = larm*cos(pi/4 + THT) - kd*sin(pi/4 + THT);
py = larm*sin(pi/4 + THT) + kd*cos(pi/4 + THT);
pz = l*sqrt(2)/2*sin(THT);
M2f = ...
     [0      0       kt       kt;
      px*kt     -px*kt     -l/2*kt    l/2*kt;
      -py*kt    py*kt      -l/2*kt    l/2*kt;
       -pz     -pz      -kd     -kd];
   
M4f = ...
    [0      0       0       0;
     px*kt     -px*kt     px*kt    -px*kt;
     -py*kt    py*kt      py*kt    -py*kt;
      -pz     -pz      pz     pz];

drone_params.control.allocation_matrix = Mu;
drone_params.control.position.kp = 2;
drone_params.control.position.kd = 1.5;
drone_params.control.velocity.kp = 2;
drone_params.control.attitude.kp = 2;
drone_params.control.attitude.kd = 0.2;
drone_params.control.attitude_rate.kp = 0.5;
drone_params.control.limits.thrust.max = 0.85*4*kt * ...
  drone_params.motors.omega_max^2;
drone_params.control.limits.thrust.min = 0.25* drone_params.mechanical.m *10;
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