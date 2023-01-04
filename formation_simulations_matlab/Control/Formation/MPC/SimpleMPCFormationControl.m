function [out, result] = SimpleMPCFormationControl(states, B_desired, B_measure, d_measure, init_guess, nDrones, N_p, dt_p, edges)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Do Optimization

drones = struct2cell(states);

for i = 1:nDrones
  if(i < drones{i}.ID)
    ID = drones{i}.ID - drones{1}.ID + 1;
  else
    ID = drones{i}.ID;
  end
  
  % set initial guess
  ig_idx = (i-1)*4*N_p+1 : i*4*N_p;
  ig = init_guess(ig_idx);
  
  % get drone_i states
  p = drones{i}.p;
  q = drones{i}.q;
  v = quat2rotm(q') * drones{i}.v;
  
  % get drone_i measurements
  Bd = [];
  p_targets = [];
  B = [];
  d = [];
  for j = 1:length(edges)
    if edges(j,1) == ID
      Bd = [Bd, B_desired(:,j)];
      B = [B, B_measure(:,j)];
      d = [d, d_measure(j)];
    end
  end
  [out_opt, result_opt] = simpleBearingMPCfnc(Bd,p_targets,B, d, p, v, q, N_p, dt_p, ig);
  
  % process results
  result(ig_idx , 1) = result_opt;
  out_idx = [((i-1)*4 +1) : i*4];
  out( out_idx, 1) = out_opt;
end
end

