function [out, result] = CommunicatingMPCFormationControl(states, B_desired, B_measure, d_measure, init_guess, nDrones, N_p, dt_p, edges)
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
  comms = [];
  for j = 1:length(edges)
    if edges(j,1) == ID
      idx = [3*(j-1)+1 : 3*j];
      Bd = [Bd; B_desired(idx,:)];
      B = [B, B_measure(:,j)];
      d = [d, d_measure(j)];

      target = edges(j,2);
      target_comm.ID = target;
      if target < 1000*ID
        target_comm.q0 = drones{target}.q;
        target_comm.v0 = drones{target}.v;
        target_comm.prediction = init_guess([(target-1)*4*N_p+1 : target*4*N_p]);
        target_comm.listen = true;
      else
        target_comm.listen = false;
      end
        comms = [comms, target_comm];
    end
  end
  [out_opt, result_opt] = communicationBearingMPCfnc(Bd,B, d, p, v, q, comms ,N_p, dt_p, ig);
  
  % process results
  result(ig_idx , 1) = result_opt;
  out_idx = [((i-1)*4 +1) : i*4];
  out( out_idx, 1) = out_opt;
end

end

