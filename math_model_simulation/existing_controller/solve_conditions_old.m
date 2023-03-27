function [result] = solve_conditions_old(thrust, moments, cond_type)
result = [];
f = thrust;
tx = moments(1);
ty = moments(2);
tz = moments(3);

% drone_params_control_conditions_unfolded_arm1 = ...
%     @(f,tx,ty,tz) 0.0143826923076923*f - 0.0421082840521321*tx - 0.0252153420017981*ty - 1.30385165048108*tz;
% drone_params_control_conditions_unfolded_arm2 = ...
%     @(f,tx,ty,tz) 0.0143826923076923*f - 0.0421082840521321*tx + 0.0252153420017981*ty + 1.30385165048108*tz;
% drone_params_control_conditions_unfolded_arm3 = ...
%     @(f,tx,ty,tz) 0.0143826923076923*f + 0.0421082840521321*tx + 0.0252153420017981*ty - 1.30385165048108*tz;
% drone_params_control_conditions_unfolded_arm4 = ...
%     @(f,tx,ty,tz) 0.0143826923076923*f + 0.0421082840521321*tx - 0.0252153420017981*ty + 1.30385165048108*tz;

% drone_params_control_conditions_folded2_arm1 = ...
%     @(f,tx,ty,tz) [f tx ty tz]*[0.0368826923076923; 0.0800735223916017; 0.0225016103144387; 0.00588074286911401];
% drone_params_control_conditions_folded2_arm2 = ...
%     @(f,tx,ty,tz) [f tx ty tz]*[0.0237363521374971; 0.344750853766948; -0.289493259493573; 1.25966226079569];
% drone_params_control_conditions_folded2_arm3 = ...
%     @(f,tx,ty,tz) [f tx ty tz]*[0.0368826923076923; -0.0800735223916017; -0.0225016103144387; 0.00588074286911401];
% drone_params_control_conditions_folded2_arm4 = ...
%     @(f,tx,ty,tz) [f tx ty tz]*[0.0237363521374971; -0.344750853766948; 0.289493259493573; 1.25966226079569];
% 
% cond_funcs_u = {...
%              drone_params_control_conditions_unfolded_arm1;
%              drone_params_control_conditions_unfolded_arm2;
%              drone_params_control_conditions_unfolded_arm3;
%              drone_params_control_conditions_unfolded_arm4;
%                  };
% cond_funcs_2f = [...
%              drone_params_control_conditions_folded2_arm1;
%              drone_params_control_conditions_folded2_arm2;
%              drone_params_control_conditions_folded2_arm3;
%              drone_params_control_conditions_folded2_arm4;
%                  ];

if cond_type == "u"
    result = 0.99*sign(tz)*(0.0143826923076923*abs(f) - 0.0421082840521321*abs(tx) - 0.0252153420017981*abs(ty))/1.30385165048108;
end

end