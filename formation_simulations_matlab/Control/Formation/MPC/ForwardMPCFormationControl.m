function [out, result] = ForwardMPCFormationControl(states, B_desired, B_measure, d_measure, init_guess, nDrones, N_p, dt_p, edges)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Do Optimization

drones = struct2cell(states);

for i = 1:nDrones
  ID = i;  
  % set initial guess
  ig_idx = (i-1)*4*N_p+1 : i*4*N_p;
  ig = init_guess(ig_idx);
  
  % get drone_i states
  p = drones{i}.p;
  q = drones{i}.q;
  v = quat2rotm(q') * drones{i}.v;
  
  % get drone_i measurements
  Bd = [];
  B = [];
  d = [];
  n_targets = 0;
  for j = 1:length(edges)
    if edges(j,1) == ID
      n_targets = n_targets +1;
      idx = [3*(j-1)+1 : 3*j];
      Bd = [Bd; B_desired(idx,:)];
      B = [B, B_measure(:,j)];
      d = [d, d_measure(j)];
    end
  end
  [out_opt, result_opt] = forwardBearingMPCfnc(Bd,B, d, p, v, q, N_p, dt_p, ig);
  
  % process results
  result(ig_idx , 1) = result_opt;
  out_idx = [((i-1)*4 +1) : i*4];
  out( out_idx, 1) = out_opt;
  
end
end

