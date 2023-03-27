% Plot data from simulation

figure(); hold on; grid on; box on;
plot(out.Drone1.p);
yline(3,'k--');
yline(0,'k--');
% yline(1,'k--');
legend("x","y","z","ref");
title("Control test")