%% Setup Workspace

% clear workspace, figures, and shell
clear all
close all
clc

% add subfolders to path
addpath("tools","format","plot","existing_controller")

% Load Drone State Bus
load("DroneStateBus.mat")

%% Set Parameters and initial conditions
% Drone Parameters
drone_params=get_parameters(1);

% % delete incorrect values
% drone_params(2).control.allocation_matrix = eye(4); % the allocation matrix is valid ONLY for the example
% drone_params(3).control.allocation_matrix = eye(4); % the allocation matrix is valid ONLY for the example
% 
% % specify that drone3 has twelve propellers
% drone_params(3).motors.omega0 = zeros(4,1); % required du to bug preventing different size arrays in struct vector
% drone_params(3).mechanical.m = 2.5;
% drone_params(3).mechanical.l = 0.5;
% drone_params(3).mechanical.I = 100*drone_params(1).mechanical.I;

% control parameters
control_params;

% define gravity
g = 9.81;

% Initial Conditions
p0 = [[0,0,2]',[0,0,2]',[0,0,2]'];
v0 = [[0,0,0]',[0,0,0]',[0,0,0]'];
q0 = [[1,0,0,0]',[1,0,0,0]',[1,0,0,0]']; %[w x y z] convention
omega0 = [[0,0,0]',[0,0,0]',[0,0,0]'];


%% Simulink Parameters
dt = 0.005;
tsim = 20;

gust_force = [0;0;0]; % force in N in world frame
gust_start = 8; % start time of wind gust
gust_duration = 5; % duration of wind gust

