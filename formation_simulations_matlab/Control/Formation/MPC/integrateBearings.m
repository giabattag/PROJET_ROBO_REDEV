function [B_] = integrateBearings(type,B0,d0,ptarget,p,v,q,omega,dt)
%integrateQuadrotorInput Does a forward integration of the quadrotor
%bearings
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

[~,Np] = size(q);
n_features = length(d0);
B_ = zeros(3*n_features, Np);

% 3D visual predictions
if strcmp(type,'3D')
  for i = 1:Np
    R = quat2rotm(q(:,i)');
    for j = 1:n_features
      feature_idx = 3*j-2 : 3*j;
    	dp = ptarget(:,j) - p(:,i);
      B_(feature_idx,i) = R' * dp/norm(dp);
    end
  end
% 2D visual predicitions
elseif strcmp(type,'2D')
  p = p -  p(:,1);
  R0 = quat2rotm(q(:,1)');
  for j = 1:n_features
    B = B0(:,j);
    d = d0(j);
    p_target_virt(:,j) = d*R0*B;
  end
  for i = 1:Np
    R = quat2rotm(q(:,i)');
    for j = 1:n_features
      feature_idx = 3*j-2 : 3*j;
    	dp = p_target_virt(:,j) - p(:,i);
      B_(feature_idx,i) = R' * dp/norm(dp);
      [Rf, ~] = build_flat_rotation(q(:,i));
      B_(feature_idx,i) = Rf' * R * B_(feature_idx,i); 
    end
  end
elseif strcmp(type,'2.5D')
  error("2.5D visual servoing not working yet")
else
  error("Choose a valide input type")
end
end

%% Multiplication of Quaternions
function q = multiplyQuaternions(qA,qB)
q0 = qA(1);
q1 = qA(2);
q2 = qA(3);
q3 = qA(4);
Q = [q0, -q1, -q2, -q3;
     q1,  q0, -q3,  q2;
     q2,  q3,  q0, -q1;
     q3, -q2,  q1,  q0];
q = Q * qB;
end

%% Build Flat Rotation
function [R, yaw] = build_flat_rotation(q)
R_full = quat2rotm(q');
z = [0;0;1];

if(abs(R_full(3,1))<abs(R_full(3,2)))
  x_proj = [R_full(1:2,1);0];
  y_proj = cross(z,x_proj);
else
  y_proj = [R_full(1:2,2);0];
  x_proj = cross(y_proj,z);
end
R = [x_proj,y_proj,z];
yaw = atan2(x_proj(2),x_proj(1));
end


%% SkewMatrix
function sk = skew(u)
  sk = zeros(3,3);
  sk(1,2) = -u(3);
  sk(1,3) = u(2);
  sk(2,1) = u(3);
  sk(2,3) = -u(1);
  sk(3,1) = -u(2);
  sk(3,2) = u(1);
end


%% RK4 function
% solves the initial value problem ydot = f(x,y)
function y = RK4(F_xy,x,y0,dt)
    [~,n] = size(x); %number of inputs
    y = zeros(length(y0),n);
    y(:,1)=y0; % initial condition
  for i=1:(n-1) % loop through inputs
    k_1 = F_xy(x(:,i),y(:,i)); %calculate k1
    k_2 = F_xy(x(:,i)+x(:,i)*0.5*dt,y(:,i)+0.5*dt*k_1); %calculate k2
    k_3 = F_xy((x(:,i)+x(:,i)*0.5*dt),(y(:,i)+0.5*dt*k_2)); %calculate k3
    k_4 = F_xy((x(:,i)+x(:,i)*dt),(y(:,i)+k_3*dt));%calculate k4
    y(:,i+1) = y(:,i) + (1/6)*(k_1+2*k_2+2*k_3+k_4)*dt; % Main calculation
  end
end
