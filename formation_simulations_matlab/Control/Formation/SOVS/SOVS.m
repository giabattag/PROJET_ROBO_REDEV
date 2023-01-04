function [out, cond_num] = SOVS(B, Bd, Bddot, Bdddot, distances, droneStates, nDrones, edges)
% Second Order Visual Bearing Controller sets the linear acceleration and yaw acceleration of each drone.

wn = 1.75;
xi = 0.75;
kp = wn*wn;
kd = 2*xi*wn;

u = zeros(3,nDrones); % linear acceleration container
omega = zeros(1,nDrones); % angular acceleration container
S = [0 -1 0; 1 0 0; 0 0 0]; % [z0]_x skew matrix

drones = struct2cell(droneStates);

% calculate relative yaws
for i = 1:nDrones
  [Ri,y] = build_flat_rotation(drones{i}.q);
  yaw(i) = y;
  R(:,:,i) = Ri;
  Ryx(:,:,i) = inv(Rzmat(y))*quat2rotm(drones{i}.q);
end
for E = 1:length(edges)
  iYj(E) = yaw(edges(E,2)) - yaw(edges(E,1));
end

% Flatten bearings
for i = 1:length(B)
  fR0 = squeeze(R(:,:,edges(i,1)))';
  oRb = quat2rotm(drones{edges(i,1)}.q');
  B(:,i) = fR0 * oRb * B(:,i);
end
cond_num = [];

for di = 1:nDrones
  Li = [];
  hi = [];
  auxi = [];
  T = eye(4); % regularisation matrix
  T(4,4) = .1;
  for E = 1:length(edges)
    if di == edges(E,1) %if the drone is observing another
      dj = edges(E,2); % index of observed drone
      % build model
      dij = distances(E);
      vi = drones{di}.v;
      wz_i = drones{di}.o(3);
      vj = drones{dj}.v;
      aj = drones{dj}.a;        
      wz_j = drones{dj}.o(3);
      iRj = Rzmat(iYj(E));
      Bij = B(:,E);
      
      P = eye(3)-Bij*Bij';
      Lij = [-P/dij, -S*Bij];
      Bijdot = [Lij, P*iRj/dij, [0;0;0]] * [vi;wz_i;vj;0];
      Pdot = -Bijdot*Bij.' - Bij*Bijdot.';
      dijdot = -Bij.'*(vi - iRj*vj);
      Lijdot = [(-Pdot*dij + P*dijdot)/(dij^2), -S*Bijdot];
      
      hijgi = Lij*[cross(vi,[0;0;wz_i]);0] + Lijdot*[vi;wz_i];
      
      Rdot = (wz_j-wz_i)*[-sin(iYj(E)), -cos(iYj(E)), 0; cos(iYj(E)), -sin(iYj(E)), 0; 0, 0, 0];
      hijgj = ((Pdot*iRj + P*Rdot)*dij - P*iRj*dijdot)*vj/(dij^2) ...
        - Lij*[cross(iRj*vj,[0;0;wz_j]);0] - Lij*[iRj*aj; 0];
      
      hij = [0;0;0];
      hij = hijgi; %+ hijgj;
    
      Li = [Li;Lij];
      hi = [hi;hij];
      % desired states
      Bijd = Bd(:,E);
      Bijddot = Bddot(:,E);
      Bijdddot = Bdddot(:,E);
      auxi = [auxi; kp*(Bijd-Bij) + kd*(Bijddot-Bijdot) + Bijdddot];
% %       auxi = [auxi; kp*(Bijd-Bij) + kd*(zeros(3)-Bijdot)];
    end % go to next edges
  end % calculate input and go to next drone i
  % Direct inversion
%   ui = pinv(Li*T)*(auxi-hi);
  cond_num(di,1)= cond(Li);
  % Singular value regularization
    %solve with regularization
  [U,Sig,V] = svd(Li*T);
  stdev = 0.025;
  mag = 0.1;
  G = eye(4);
  for i=1:rank(rank(Sig))
    G(i,i) = mag * exp(-1.0/2 * (Sig(i,i)/stdev)^2 );
  end
  ui = V*pinv(Sig.'*Sig + G)*Sig.'*U.'*(auxi-hi);
  cond_num(di,2)=cond(V*pinv(Sig.'*Sig + G)*Sig.'*U.');
  
  u_null = SOVS_nullspace_control(drones, nDrones, [-1;2;1], 2);
  
  u(:,di) = ui(1:3) + u_null(1:3,di);
%   u(:,di) =  u_null(1:3,di);
  omega(di) = ui(4);
  u(:,di) = u(:,di);% + quat2rotm(drones{di}.q')' * [0;0;0];
end
out = reshape([u;omega],[4*nDrones,1]);
end


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



