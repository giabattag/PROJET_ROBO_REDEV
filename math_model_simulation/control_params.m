% Control parameters for drone control laws

%% PD

global kp_position_pid kd_position_pid kp_velocity_pid kp_attitude_pid kd_attitude_pid kp_attituderate_pid

kp_position_pid = 2;
kd_position_pid = 1.5;
kp_velocity_pid = 2;
kp_attitude_pid = 2;
kd_attitude_pid = 0.2;
kp_attituderate_pid = 0.5;


%% SMC

global kp_z_smc kd_z_smc kp_x_smc kd_x_smc kp_y_smc kd_y_smc k1_attitude_smc k2_attitude_smc k3_attitude_smc tuning_parameter_smc

kp_z_smc = 4;
kd_z_smc = 4;
kp_x_smc = 20;
kd_x_smc = 30;
kp_y_smc = 10;
kd_y_smc = 10;

k1_attitude_smc = 55;
k2_attitude_smc = 55;
k3_attitude_smc = 55;

tuning_parameter_smc = 100;

%% Supertwisting

global kp_z_stw kd_z_stw kp_x_stw kd_x_stw kp_y_stw kd_y_stw k11_attitude_stw k12_attitude_stw k13_attitude_stw ...
     k21_attitude_stw k22_attitude_stw k23_attitude_stw tuning_parameter_stw

kp_z_stw = 4;
kd_z_stw = 4;
kp_x_stw = 20;
kd_x_stw = 30;
kp_y_stw = 10;
kd_y_stw = 10;


dDeltaM = 10;
k11_attitude_stw = 1.5*sqrt(dDeltaM);
k12_attitude_stw = 1.5*sqrt(dDeltaM);
k13_attitude_stw = 1.5*sqrt(dDeltaM);
k21_attitude_stw = 1.1*dDeltaM;
k22_attitude_stw = 1.1*dDeltaM;
k23_attitude_stw = 1.1*dDeltaM;

tuning_parameter_stw = 100;
