%% MultiQuadInit.m
% Initialisation Script for the simulation of a formation of multiple quadrotors.
%
% Sets up simulation parameters, runs ErskineMultiDroneFormation.slx simulink file
% and plots the drone flights. 
%
% The examples here use bearing formation control, but the structure should
% work equally well for other types of control.
%
%
% Author: Julian Erskine
% Address: LS2N, Ecole Centrale de Nantes
% Email: julian.erskine@ls2n.fr
% Last revision: 17-03-2020

%% Preliminaries
clear all
close all
clc
addpath(genpath(fileparts(which(mfilename))));


%% Simulation Params
disp("Initializing Sim Parameters")
sim_dt = 0.005
Tsim = 25
controller_rate = 50;
% controller_rate = 200;

%% Setup Problem

% Number of drones if formation
nDrones = 3;

% set parameters
drone_params = setDefaultDroneParams(nDrones);
g = 9.81;


% Initial States  
scale = 3
p0 = scale*(rand(3,nDrones) - [0.5;0.5;0])
v0 = zeros(3, nDrones);
a0 = zeros(3, nDrones);
q0 = zeros(4,nDrones); q0(1,:) = ones(1,nDrones);
omega0 = zeros(3,nDrones);

% Sim Key
simkey.p = [1:3];
simkey.v = [4:6];
simkey.a = [7:9];
simkey.q = [10:13];


% camera noise
camera_noise.perp = 0.05;
camera_noise.long = 0.1;
camera_noise.long_bias = 0.05;

load('DroneStateBus');
FormationStateBus = generateFormationStateBus(nDrones);


%% Bearing Target
edges = [1,2; 
  1,3;
  2,1;
  2,3;
  3,1;
  3,2];

% edges = [1,2; 
%  1,3;
%   %1,4;
%   1,5;
%   2,1;
%   2,3;
%   %2,4;
%   2,5;
%   3,1;
%   %3,2;
%   3,4;
%   %3,5;
%   4,5;
%   %4,2;
%   4,3;
%   5,4;
%   %5,1;
%   5,2
%   5,3];
% % edges = [1,2; 
% %   1,3;
% %   2,1;
% %   3,2];
T = [0,0.1,Tsim-1];

% create desired formation close to initial conditions:
%pf = p0(:,1:nDrones) + scale*(rand(3, nDrones)-[0.5; 0.5; 0])
% create random desired formation
% pf = scale*(rand(3, nDrones)-[0.5; 0.5; 0]);
pf = scale*(rand(3, nDrones)-[0.5; 0.5; 0]);
% pf = p0+0.5*(rand(3, nDrones)-0.5);
P = [[p0(:,1:nDrones),p0(:,1:nDrones),pf];[zeros(1,nDrones),zeros(1,nDrones),pi/2*(rand(1,nDrones)-0.5)]];

PTS = trajectory_interpolation(P,T,sim_dt,'step');

for i=1:length(PTS.time)
  Qt = reshape(PTS.data(i,:),[4,nDrones]);
  B(i,:) = reshape(calculate_bearings(edges,Qt(1:3,:),Qt(4,:)), [1,3*length(edges)]);
  [~,~,rigidity(i,1)] = build_bearing_rigidity_matrix(Qt(1:3,:),Qt(4,:),edges);
end

Bd_traj = timeseries(B,PTS.time);
Rigidity_traj = timeseries(rigidity,PTS.time);
plot(Rigidity_traj)
% Create futur trajectory reference, used for MPC. Values needed to run,
% but non-essential otherwise.
N_p = 5
dt_p = 0.2
rate_mpc = 25
for i = 1:length(PTS.time)
  for j = 1:N_p
    t = Bd_traj.Time(i) + j*dt_p;
    if t >= Bd_traj.TimeInfo.End
      t = Bd_traj.TimeInfo.End;
    end
    idx = find(round(Bd_traj.Time,3) == round(t,3));
    Bforward(:,j,i) = Bd_traj.Data(idx,:)';
  end
end
Bd_future = timeseries(Bforward,PTS.Time)


% %% Run simulation
% return
% disp("Running Simulation")
% out = sim("ErskineMultiDroneFormation.slx");
% disp("Plotting Simulation")
% plotDroneFlights(out,simkey, ['r','b','k']);
% plotResults(out,Bd_traj,Rigidity_traj)

return
%% Collect Data For Comparison
% Setup Sim generation
nSims = 200;
model = "ErskineMultiDroneFormation_5drones";
load_system(model)
% setup result storage
savefolder = [string('multiSimResults/')+string(date)+"-"+string(randi([1000,9999])+"/")]
if ~exist(savefolder, 'dir')
  mkdir(savefolder)
end
multiSimResults = [];

start_time = tic;
for s = 1:nSims
  sim_start_time = tic;
  disp(["Running sim #"+num2str(s)])
  setSimConditions;
  out = sim(model);
  multiSimResults = [multiSimResults, out];
  savename = string("sim")+string(s);
  save(savefolder+savename, 'out', 'Bd_traj', 'Rigidity_traj');
  sim_end_time = toc(sim_start_time);
  disp(["Sim #"+num2str(s)+" finished in "+num2str(sim_end_time)+" s"])
end
end_time = toc(start_time)
disp('Finished')

return
%% Plot Values and 
% figure('Position',  [0, 0, 500, 250])
% yyaxis left
% plot(out.SOVS.bearing_error)
% xlabel("Time (s)")
% ylabel("||{\bf e}_\beta||")
% yyaxis right
% plot(out.SOVS_cond)
% ylabel("\kappa")
% title("")

figure('Position',  [0, 0, 400, 200]); hold on;
plot(out.SOVS.bearing_error)
plot(out.Gradient.bearing_error)
xlabel("Time (s)")
ylabel("||{\bf e}_\beta||")
title("")
legend(['SOVS control',"Rigidity Control"])


figure('Position',  [0, 0, 400, 200]); hold on;
plot(out.SOVS_d.Time, squeeze(out.Grad_d.Data(1,1,:)))
plot(out.SOVS_d.Time, squeeze(out.Grad_d.Data(1,2,:)))
plot(out.SOVS_d.Time, squeeze(out.Grad_d.Data(1,4,:)))
xlabel("Time (s)")
ylabel("distance (m)")
title("")
legend(['d_{12}',"d_{13}","d_{23}"])


figure('Position',  [0, 0, 400, 200]); hold on;
sig = 0:0.01:0.5
p = 0.1*exp(-sig.^2/0.02^2)
ureg = 1./sig
reg1 = sig./(sig.^2+p)
reg2 = sig./(sig.^2+0.025)
plot(sig, ureg,'linewidth',2)
plot(sig, reg1,'linewidth',2)
plot(sig, reg2,'linewidth',2)
ylim([0,20])
xlabel("\sigma({\bf L}_i)")
ylabel("Singular value")
legend("1/\sigma","\sigma/(\sigma^2+\gamma)","\sigma/(\sigma^2+r(\sigma))")