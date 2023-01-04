function [p_,v_,a_,q_] = integrateQuadrotorInput(type,u,p0,v0,a0,q0,om0,dt)
%integrateQuadrotorInput Does a single step forward integration of the
%quadrotor state
%
%   integrateQuadrotorInput(type,u,p,v,a,q,om)
%     type: "velocity_yawrate"
%           "thrust_attitude"
%           "thrust_bodyrate"
%           "motor_speeds"
%     u: input vector
%     p: estimated world position
%     v: body_frame velocity
%     a: body frame acceleration
%     q: world to quadrotor attitude
%     om: quadrotor body rate

mass = 1.0; %TODO add to arguments
g = 9.81;

[~,n] = size(u);
p_ = zeros(3,n);
v_ = zeros(3,n);
a_ = zeros(3,n);
q_ = zeros(4,n);
om_ =zeros(3,n);
th_ =zeros(1,n);

if strcmp(type,'velocity_yawrate')
elseif strcmp(type,'thrust_attitude')
elseif strcmp(type, 'thrust_bodyrate')
  om_ = u(2:4,:); % output omega is reference
  th_ = u(1,:);
  
  Qdot = @(o,q) 0.5 * multiplyQuaternions([0;o],q);
  q_ = RK4(Qdot,om_,q0,dt);
  
  %xdd = R*[0;0;f]/m - [0;0;g]
  for i =1:n
    a_(:,i) = quat2rotm(q_(:,i)')*[0;0;th_(i)]/mass -[0;0;g];
  end
  Xdot = @(X,y) X;
  v_ = RK4(Xdot,a_,v0,dt);
  p_ = RK4(Xdot,v_(:,1:length(v_)-1),p0,dt);

  
elseif strcmp(type, 'motor_speeds')
else
  error("Choose a valide input type")
end
end


%% RK4 function
% solves the initial value problem ydot = f(x,y)
function y = RK4(F_xy,x,y0,dt)
    [~,n] = size(x); %number of inputs
    y = zeros(length(y0),n+1);
    y(:,1)=y0; % initial condition
  for i=1:(n) % loop through inputs %%%%%% Change to n
    k_1 = F_xy(x(:,i),y(:,i)); %calculate k1
    k_2 = F_xy(x(:,i)+x(:,i)*0.5*dt,y(:,i)+0.5*dt*k_1); %calculate k2
    k_3 = F_xy(x(:,i)+x(:,i)*0.5*dt,y(:,i)+0.5*dt*k_2); %calculate k3
    k_4 = F_xy((x(:,i)+x(:,i)*dt),(y(:,i)+k_3*dt));%calculate k4
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*dt; % Main calculation
  end
end
