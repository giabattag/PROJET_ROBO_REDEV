function [u,result] = forwardBearingMPCfnc(Bd, B0, d0, p0, v0, q0, Np, dt_p, initial_guess)
%POSITIONMPCFNC Summary of this function goes here
%   Detailed explanation goes here

% Setup optimization function handles
ofun3D = @(x)objFunction(x, Bd, B0, d0, p0, v0, q0, Np, dt_p);
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
  'MaxFunctionEvaluations',5*Np*4,...
  'OptimalityTolerance',1e-1,...
  'StepTolerance',1e-1);
% Run optimization
result = fmincon(ofun3D,x0,A,b,Aeq,beq,lb,ub,nonlcon,options);

% Process Optimization Results
%TODO

u = result(1:4,1);
end

%% Objective Function
function [cost] = objFunction(u, Bd, B0, d0 , p0, v0, q0, Np, dt_p)

[n_targets,~] = size(Bd);
n_targets = n_targets/3;

input = reshape(u,[4,Np]);
omega = input(2:4,Np);


[p,vt,~,q] = integrateQuadrotorInput("thrust_bodyrate",input,p0,v0,0,q0,0,dt_p);

[Bt] = integrateBearings("2D",B0,d0, [] ,p,vt,q, omega, dt_p);

kd = 1.1;

kmax = 1.4;
ecut = 0.15;
kmin = 1.05;

Bd = Bd;
Bt = Bt;
n_targets = n_targets;
k_adapt = kmin + ((kmax-kmin)/ecut) * norm(Bd(:,1) - Bt(:,1))/n_targets;

if k_adapt > kmax
  k_adapt = kmax;
end
if k_adapt < 1.2
  k_adapt = 1.2;
end
k_adapt = k_adapt;

cost = 0;
for i = 1:Np
  ep = Bd(:,i) - Bt(:,i);
  ev = [3;5;4] - vt(:,i);

  cost = cost + ep'*k_adapt^i*eye(3*n_targets)*ep + 0.2/k_adapt^4*ev'*kd^i*eye(3)*ev;
end
% kd = 1.1;
% 
% kmax = 1.4;
% kmax_v = 1.2;
% 
% ecut = 0.1;
% ecut_v = 1;
% kmin = 1.1;
% 
% vd = [3;5;4];
% 
% k_adapt_b = kmin + ((kmax-kmin)/ecut) * norm(Bd(:,1) - Bt(:,1))/n_targets;
% k_adapt_v = kmin + ((kmax_v-kmin)/ecut_v) * norm(vd - vt(:,1));
% v_weight = (norm(vd - vt(:,1))/norm((Bd(:,1) - Bt(:,1))/n_targets))/4
% % bearing adaptative gain
% if k_adapt_b > kmax
%   k_adapt_b = kmax;
% end
% if k_adapt_b < 1.15
%   k_adapt_b = 1.15;
% end
% % velocity adaptative gain
% if k_adapt_v > kmax
%   k_adapt_v = kmax;
% end
% if k_adapt_v < 1.1
%   k_adapt_v = 1.1;
% end
% cost = 0;
% for i = 1:Np
%   ep = Bd(:,i) - Bt(:,i);
%   ev = vd - vt(:,i);
%     cost = cost + ep'*k_adapt_b^i*ep + v_weight*ev'*1.2^i*ev;
% end


end

%% Constraint Function
%TODO