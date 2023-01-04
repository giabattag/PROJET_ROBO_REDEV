function [B_] = integrateBearingsWithCommunication(B0,d0, pt, qt, target_predictions)
%integrateQuadrotorInput Does a forward integration of the quadrotor
%bearings
%
%     B0: initial Bearing measurement
%     d0: initial distance measurement
%     pt: estimate of quadrotor position in the world frame centered at p0
%     qt: world to quadrotor attitude
%     target_predictions: structure containing predicted position of target
%     j in world frame centered at p0j

[~,Np] = size(pt);
n_features = length(d0);
B_ = zeros(3*n_features, Np);

% 2D visual predicitions

R0 = quat2rotm(qt(:,1)');
for j = 1:n_features
  B = B0(:,j);
  d = d0(j);
  p_target_virt0(:,j) = d*R0*B;
end
for i = 1:Np
  R = quat2rotm(qt(:,i)');
  for j = 1:n_features
    feature_idx = 3*j-2 : 3*j;
    p_target_virt = p_target_virt0(:,j) + target_predictions(j).pt(:,i);
    dp = p_target_virt - pt(:,i);
    B_(feature_idx,i) = R' * dp/norm(dp);
    [Rf, ~] = build_flat_rotation(qt(:,i));
    B_(feature_idx,i) = Rf' * R * B_(feature_idx,i);
  end
end


% end of function
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

