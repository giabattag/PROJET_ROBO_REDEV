function [u,result] = simpleBearingMPCfnc(Bd, p_targets, B0, d0, p0, v0, q0, Np, dt_p, initial_guess)
%POSITIONMPCFNC Summary of this function goes here
%   Detailed explanation goes here

% Setup optimization function handles
ofun3D = @(x)objFunction3D(x, Bd, p_targets, B0, d0, p0, v0, q0, Np, dt_p);
nonlcon = [];

% Set control signal initial condition and limits
x0 = initial_guess;
fmin = 4*ones(1,Np);
wmin = -2*ones(3,Np);
lb = reshape([fmin;wmin],[4*Np,1]);

fmax = 18*ones(1,Np);
wmax = 2*ones(3,Np);
ub = reshape([fmax;wmax],[4*Np,1]);

%Set inequat
A = [];
b = [];
Aeq = [];
beq = [];

% Set optimization parameters
options = optimoptions('fmincon','Display','off','algorithm','sqp',...
  'MaxFunctionEvaluations',100*Np*4,...
  'OptimalityTolerance',1e-1,...
  'StepTolerance',1e-1);
% Run optimization
result = fmincon(ofun3D,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

% Process Optimization Results
%TODO

u = result(1:4,1);
end

%% Objective Function
function [cost] = objFunction3D(u, Bd, p_targets, B0, d0 , p0, v0, q0, Np, dt_p)

[~,n_targets] = size(Bd);

input = reshape(u,[4,Np]);

[p,v,~,q] = integrateQuadrotorInput("thrust_bodyrate",input,p0,v0,0,q0,0,dt_p);

[Bt] = integrateBearings("2D",B0,d0, p_targets,p,v,q, [], dt_p);

kp = 1.2;
kd = 1.0;

cost = 0;
Bd_vec = reshape(Bd,[3*n_targets,1]);
Bt1 = Bt(:,1);
eB = Bd_vec - Bt1;
for i = 1:Np
  ep = Bd_vec - Bt(:,i);
  ev = [0;0;0] - v(:,i);

  cost = cost + ep'*kp^i*eye(3*n_targets)*ep + ev'*kd^i*eye(3)*ev;% + eq'*kd^i*eye(4)*eq;
end

end

%% Constraint Function
%TODO