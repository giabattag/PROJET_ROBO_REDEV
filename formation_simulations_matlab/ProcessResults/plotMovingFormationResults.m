function [] = plotResults(out, Bd_ts, Rigid_ts)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% Plot Bearing Trajectory

sim_fields = out.who
listOfControllers = [];
controllerNames =[]
for i = 1:length(sim_fields)
  if ~strncmp(sim_fields{i},'Drone',5) && ~strncmp(sim_fields{i},'tout',4)
    listOfControllers = [listOfControllers, out.get(sim_fields{i})];
    controllerNames = [ controllerNames, string(sim_fields{i})];
    disp(["Found "+sim_fields{i}])
  end
end
controllerNames
nControllers = length(listOfControllers)
time = round(out.get("tout"),3);

endidx = length(time);

if time(endidx) > Bd_ts.TimeInfo.End
  Bd_ts = addsample(Bd_ts,'Data',Bd_ts.Data(Bd_ts.TimeInfo.Length,:),'Time',time(endidx));
end
if time(endidx) > Rigid_ts.TimeInfo.End
  Rigid_ts = addsample(Rigid_ts,'Data',Rigid_ts.Data(Rigid_ts.TimeInfo.Length,:),'Time',time(endidx));
end

hResultsFig = figure('Name',"SimResults", 'Position',[200,200,600,400]);
subplot(9,1,1:4); grid on; hold on;
ax = gca;
for i = 1:nControllers
  plot(listOfControllers(i).bearing_error);
end
legend(controllerNames);
title('Bearing Error');
ylabel('Bearing Error Magnitude');
ax.YAxis.Limits(1) = 0;
subplot(9,1,6:9); grid on; hold on;
ax = gca;
for i = 1:nControllers
  plot(Bd_ts)
end
ylabel('Desired Bearing Values')
yyaxis right
plot(Rigid_ts, 'r', 'linewidth',3);
ylabel('Rigidity Eigenvalue');
title('Desired Bearings');
xlabel("Time (s)");

ax.YAxis(2).Color = [1,0,0]
ax.YAxis(2).Limits = [0, round(max(Rigid_ts.Data)+0.05,1)]

hVelFig = figure('Name', 'Velocities','DefaultAxesPosition', [0.025, 0.025, 0.975, 0.975])
subplot_tight(nControllers+1,1,1,0.065); grid on; hold on;
for i = 1:nControllers
  plot(listOfControllers(i).bearing_error)
end
legend(controllerNames)
ylabel('Bearing Error Magnitude')
for i = 1:nControllers
  controller_struct = out.get(controllerNames(i));
  controller_cell = struct2cell(controller_struct.signal1);
  p = [];
  q = [];
  v_worldframe_x = [];
  v_worldframe_y = [];
  v_worldframe_z = [];
  for j = 1:length(controller_cell)
    p = [p;squeeze(controller_cell{j}.p.Data)];
    vtmp = squeeze(controller_cell{j}.v.Data);
    qtmp = squeeze(controller_cell{j}.q.Data);
    v_world_tmp = [];
    for t = 1:length(controller_cell{j}.p.Data)
      R = quat2rotm(qtmp(:,t));
      v_world_tmp = [v_world_tmp, R*vtmp(:,t)];
    end
    v_worldframe_x = [v_worldframe_x;v_world_tmp(1,:)];
    v_worldframe_y = [v_worldframe_y;v_world_tmp(2,:)];
    v_worldframe_z = [v_worldframe_z;v_world_tmp(3,:)];
  end
  vx = v_worldframe_x;
  
  vxm = mean(abs(3-v_worldframe_x));
  vy = v_worldframe_y;
  vym = mean(abs(5-v_worldframe_y));
  vz = v_worldframe_z;
  vzm = mean(abs(4-v_worldframe_z));
  subplot_tight(nControllers+1,1,i+1,0.065); grid on; hold on;
  plot(controller_cell{1}.p.Time,vx, '--r');
  plot(controller_cell{1}.p.Time,vy, '--g');
  plot(controller_cell{1}.p.Time,vz, '--b');
  plot(controller_cell{1}.p.Time,vxm, 'r','linewidth',2);
  plot(controller_cell{1}.p.Time,vym, 'g','linewidth',2);
  plot(controller_cell{1}.p.Time,vzm, 'b','linewidth',2);
  title("Controller: "+controllerNames(i));
  ylim([0,5.5])
  ylabel("velocity ms^{-1}")
end

% end of function
end

