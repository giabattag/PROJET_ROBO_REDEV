function [u,result] = positionMPCfnc(pd, p, v, q, Np, dt_p, initial_guess)
%POSITIONMPCFNC Summary of this function goes here
%   Detailed explanation goes here

% Setup optimization function handles
ofun = @(x)objFunction(x, p, v, q, pd, Np, dt_p);
nonlcon = [];

% Set control signal initial condition and limits
x0 = initial_guess;
fmin = 4*ones(1,Np);
wmin = -1*ones(3,Np);
lb = reshape([fmin;wmin],[4*Np,1]);

fmax = 18*ones(1,Np);
wmax = 1*ones(3,Np);
ub = reshape([fmax;wmax],[4*Np,1]);

%Set inequat
A = [];
b = [];
Aeq = [];
beq = [];

% Set optimization parameters
options = optimoptions('fmincon','Display','off',...
  'MaxFunctionEvaluations',100*Np*4,...
  'OptimalityTolerance',1e-2,...
  'StepTolerance',1e-2);
% Run optimization
% result = fmincon(ofun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);
% 
% 
result = zeros(4*Np,1);
u = result(1:4,1);
end

%% Objective Function
function [cost] = objFunction(u, p0, v0, q0, pd, Np, dt_p)

input = reshape(u,[4,Np]);

[p,v,a,q] = integrateQuadrotorInput("thrust_bodyrate",input,p0,v0,0,q0,0,dt_p);

kp = 1.25;
kd = 1.0;

cost = 0;
for i = 1:Np
  ep = pd - p(:,i);
  ev = [0;0;0] - v(:,i);
  eq = [1;0;0;0] - q(:,i);

  cost = cost + ep'*kp^i*eye(3)*ep + ev'*kd^i*eye(3)*ev;% + eq'*kd^i*eye(4)*eq;
end

end

%% Constraint Function
%TODO