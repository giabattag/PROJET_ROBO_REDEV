function [u] = SOVS_nullspace_control(drones, nDrones, vd, sd)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

u = zeros(4, nDrones);

c = zeros(3,1);
for i=1:nDrones
  c = drones{i}.p/nDrones;
end

scale = [];
for i=1:nDrones
  scale = [scale, norm(c-drones{i}.p)];
end

scale = mean(scale);
e_scale = sd - scale

for i=1:nDrones
  Ri = quat2rotm(drones{i}.q');
  s_error = -e_scale * e_scale^2 * (c - drones{i}.p); % scale error in world frame
  vdi = vd;% + 0.5*s_error;
  v_error = vdi - Ri*drones{i}.v; % velocity error in world frame
  u(1:3,i) = Ri.'*(v_error);
end 
% u_null = u

end

