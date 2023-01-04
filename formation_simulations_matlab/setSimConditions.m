% Initial States  
scale = 5;
p0 = scale*(rand(3,nDrones) - [1/2;1/2;0]);
v0 = zeros(3, nDrones);
a0 = zeros(3, nDrones);
q0 = zeros(4,nDrones); q0(1,:) = ones(1,nDrones);
omega0 = zeros(3,nDrones);


edges = [];
T = [];
pf = [];
P = [];
PTS = [];
Qt = [];
B = [];
rigidity = [];
Bforward = [];

%% Bearing Target
edges = [1,2; 
  1,3;
  2,1;
  2,5;
  3,2;
  3,4;
  4,5;
  4,1;
  5,4;
  5,1;
  4,2];
% edges = [1,2; 
%   1,3;
%   2,1;
%   2,3;
%   3,1;
%   3,2];
% edges = [1,2; 
%   1,3;
%   2,1;
%   3,2];
T = [0,Tsim-1];
pf = p0;

% create desired formation close to initial conditions:
%pf = p0(:,1:nDrones) + scale*(rand(3, nDrones)-[0.5; 0.5; 0])
% create random desired formation
pf = scale*(rand(3, nDrones)-[0.5; 0.5; 0]);
P = [[p0(:,1:nDrones),pf];[zeros(1,nDrones),2*pi*(rand(1,nDrones)-0.5)]];

PTS = trajectory_interpolation(P,T,0.01,'fifth');

for i=1:length(PTS.time)
  Qt = reshape(PTS.data(i,:),[4,nDrones]);
  B(i,:) = reshape(calculate_bearings(edges,Qt(1:3,:),Qt(4,:)), [1,3*length(edges)]);
  [~,~,rigidity(i,1)] = build_bearing_rigidity_matrix(Qt(1:3,:),Qt(4,:),edges);
end

Bd_traj = timeseries(B,PTS.time);
Rigidity_traj = timeseries(rigidity,PTS.time);

% Create futur trajectory reference, used for MPC. Values needed to run,
% but non-essential otherwise.
N_p = 5;
dt_p = 0.2;
rate_mpc = 25;
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
Bd_future = timeseries(Bforward,PTS.Time);
