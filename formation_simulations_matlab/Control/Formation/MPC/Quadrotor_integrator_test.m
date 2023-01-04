close all
clc

%% Set Test parameters
time_test = false
plot_attitude = false
plot_position = true



% Get baseline values
t = out.Quad1.Time';
p_sim = out.Quad1.Data(:,1:3)';
v_sim = out.Quad1.Data(:,4:6)';
om = out.Quad1.Data(:,14:16)';
q_sim = out.Quad1.Data(:,10:13)';
thrust_sim = out.Quad1.Data(:,17)';

p0 = p_sim(:,1);
v0 = v_sim(:,1);
q0 = q_sim(:,1);

% set control noise
om = om + 0.0*(rand(size(om))-0.5); % noise


%% Do Time Test
if time_test
  n = length(t);
  factor = 20;
  dt = sim_dt*factor;
  for i = 1:1000
    [~,~,~,~] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],0,0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  end
end

%% plot attitude
if plot_attitude
  %My Attitude model at 0.005s
  [~,~,~,q_approx] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim;om],p0,v0,0,q_sim(:,1),[0;0;0],sim_dt);
  figure(); hold on; grid on;
  plot(1,1,'k',1,1,'k--')
  plot(t,q_sim)
  plot(t,q_approx,'--')
  title("full time step solution")
  
  %My Attitude model at 5 dt
  n = length(t);
  factor = 5
  [~,~,~,q_approx] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;
  plot(t,q_sim)
  plot(0:factor*sim_dt:t(n),q_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  
  %My Attitude model at 10 dt
  n = length(t);
  factor = 10
  [~,~,~,q_approx] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;
  plot(t,q_sim)
  plot(0:factor*sim_dt:t(n),q_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  
  %My Attitude model at 20 dt
  n = length(t);
  factor = 20
  [~,~,~,q_approx] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;
  plot(t,q_sim)
  plot(0:factor*sim_dt:t(n),q_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  
  %My Attitude model at 40 dt
  n = length(t);
  factor = 40
  [~,~,~,q_approx] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;
  plot(t,q_sim)
  plot(0:factor*sim_dt:t(n),q_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
end


%% Plot Position
if plot_position
  %My Attitude model at 0.005s
  [p_approx,~,~,~] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim;om],p0,v0,0,q_sim(:,1),[0;0;0],sim_dt);
  figure(); hold on; grid on;
  plot(1,1,'k',1,1,'k--')
  plot(t,p_sim)
  plot(t,p_approx,'--')
  title("full time step solution")
  legend(["Truth","Approx"]);
  
  %My Attitude model at 5 dt
  n = length(t);
  factor = 5
  [p_approx,~,~,~] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;  plot(1,1,'k',1,1,'k--')
  plot(t,p_sim)
  plot(0:factor*sim_dt:t(n),p_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  legend(["Truth","Approx"]);

    %My Attitude model at 10 dt
  n = length(t);
  factor = 10
  [p_approx,~,~,~] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;  plot(1,1,'k',1,1,'k--')
  plot(t,p_sim)
  plot(0:factor*sim_dt:t(n),p_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  legend(["Truth","Approx"]);

  %My Attitude model at 20 dt
  n = length(t);
  factor = 20
  [p_approx,~,~,~] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;   plot(1,1,'k',1,1,'k--')
  plot(t,p_sim)
  plot(0:factor*sim_dt:t(n),p_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  legend(["Truth","Approx"]);

  %My Attitude model at 40 dt
  n = length(t);
  factor = 40
  [p_approx,~,~,~] = integrateQuadrotorInput("thrust_bodyrate",[thrust_sim(1:factor:n);om(:,1:factor:n)],p0,v0,0,q_sim(:,1),[0;0;0],factor*sim_dt);
  figure(); hold on; grid on;   plot(1,1,'k',1,1,'k--')
  plot(t,p_sim)
  plot(0:factor*sim_dt:t(n),p_approx,'--')
  title(["reduced time step solution. dt=",num2str(factor*sim_dt)])
  legend(["Truth","Approx"]);

end