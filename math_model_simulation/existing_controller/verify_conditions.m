function [verify_u, verify_2f, conditions_u,conditions_2f] = verify_conditions(thrust,moments,omega_props)

f = abs(thrust);
tx = abs(moments(1));
ty = abs(moments(2));
tz = abs(moments(3));

drone_params_control_conditions_unfolded_arm1 = ...
    @(f,tx,ty,tz) 0.0143826923076923*f - 0.0421082840521321*tx - 0.0252153420017981*ty - 1.30385165048108*tz;
drone_params_control_conditions_unfolded_arm2 = ...
    @(f,tx,ty,tz) 0.0143826923076923*f - 0.0421082840521321*tx + 0.0252153420017981*ty + 1.30385165048108*tz;
drone_params_control_conditions_unfolded_arm3 = ...
    @(f,tx,ty,tz) 0.0143826923076923*f + 0.0421082840521321*tx + 0.0252153420017981*ty - 1.30385165048108*tz;
drone_params_control_conditions_unfolded_arm4 = ...
    @(f,tx,ty,tz) 0.0143826923076923*f + 0.0421082840521321*tx - 0.0252153420017981*ty + 1.30385165048108*tz;

drone_params_control_conditions_folded2_arm1 = ...
    @(f,tx,ty,tz) [f tx ty tz]*[0.0368826923076923; 0.0800735223916017; 0.0225016103144387; 0.00588074286911401];
drone_params_control_conditions_folded2_arm2 = ...
    @(f,tx,ty,tz) [f tx ty tz]*[0.0237363521374971; 0.344750853766948; -0.289493259493573; 1.25966226079569];
drone_params_control_conditions_folded2_arm3 = ...
    @(f,tx,ty,tz) [f tx ty tz]*[0.0368826923076923; -0.0800735223916017; -0.0225016103144387; 0.00588074286911401];
drone_params_control_conditions_folded2_arm4 = ...
    @(f,tx,ty,tz) [f tx ty tz]*[0.0237363521374971; -0.344750853766948; 0.289493259493573; 1.25966226079569];

conditions_u = [...
             drone_params_control_conditions_unfolded_arm1(f,tx,ty,tz) > 0;
             drone_params_control_conditions_unfolded_arm2(f,tx,ty,tz) > 0;
             drone_params_control_conditions_unfolded_arm3(f,tx,ty,tz) > 0;
             drone_params_control_conditions_unfolded_arm4(f,tx,ty,tz) > 0;
             omega_props > 0;
                 ];
conditions_2f = [...
             drone_params_control_conditions_folded2_arm1(f,tx,ty,tz) > 0;
             drone_params_control_conditions_folded2_arm2(f,tx,ty,tz) > 0;
             drone_params_control_conditions_folded2_arm3(f,tx,ty,tz) > 0;
             drone_params_control_conditions_folded2_arm4(f,tx,ty,tz) > 0;
             omega_props(1:2) < 0;
             omega_props(3:4) > 0;
                 ];

verify_u = (sum(conditions_u) == 8);
verify_2f = (sum(conditions_2f) == 8);

end


