function [result] = solve_conditions(thrust, moments, cond_type)
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
%     @(f,tx,ty,tz) abs([f tx ty tz])*[0.0368826923076923; 0.0800735223916017; 0.0225016103144387; 0.00588074286911401];
% drone_params_control_conditions_folded2_arm2 = ...
%     @(f,tx,ty,tz) abs([f tx ty tz])*[0.0237363521374971; 0.344750853766948; -0.289493259493573; 1.25966226079569];
% drone_params_control_conditions_folded2_arm3 = ...
%     @(f,tx,ty,tz) abs([f tx ty tz])*[0.0368826923076923; -0.0800735223916017; -0.0225016103144387; 0.00588074286911401];
% drone_params_control_conditions_folded2_arm4 = ...
%     @(f,tx,ty,tz) abs([f tx ty tz])*[0.0237363521374971; -0.344750853766948; 0.289493259493573; 1.25966226079569];


if cond_type == "u"
    result = 0.7*sign(tz)*(0.0143826923076923*abs(f) - 0.0421082840521321*abs(tx) - 0.0252153420017981*abs(ty))/1.30385165048108;
elseif cond_type == "2f"
    
%     result = fmincon(@(x) x(1)^2 + x(2)^2 + x(3)^2, [tx; ty; tz], [], [], [], [], [-0.1; -0.1; -0.1], [0.1; 0.1; 0.1], @nonlcon, ...
%                 optimoptions("fmincon","Display","none"));
    result = fmincon(@(x) (x(1) - tz)^2, tz, [], [], [], [], -0.1, 0.1, @nonlcon, ...
                optimoptions("fmincon","Display","none"));

end


function [c, ceq] = nonlcon(x)
    drone_params_control_conditions_folded2_arm1 = ...
        @(f,tx,ty,tz) abs([f tx ty tz])*[0.0368826923076923; 0.0800735223916017; 0.0225016103144387; 0.00588074286911401];
    drone_params_control_conditions_folded2_arm2 = ...
        @(f,tx,ty,tz) abs([f tx ty tz])*[0.0237363521374971; 0.344750853766948; -0.289493259493573; 1.25966226079569];
    drone_params_control_conditions_folded2_arm3 = ...
        @(f,tx,ty,tz) abs([f tx ty tz])*[0.0368826923076923; -0.0800735223916017; -0.0225016103144387; 0.00588074286911401];
    drone_params_control_conditions_folded2_arm4 = ...
        @(f,tx,ty,tz) abs([f tx ty tz])*[0.0237363521374971; -0.344750853766948; 0.289493259493573; 1.25966226079569];

    c = [
%             -drone_params_control_conditions_folded2_arm1(f,x(1),x(2),x(3));
%             -drone_params_control_conditions_folded2_arm2(f,x(1),x(2),x(3));
%             -drone_params_control_conditions_folded2_arm3(f,x(1),x(2),x(3));
%             -drone_params_control_conditions_folded2_arm4(f,x(1),x(2),x(3));
            -drone_params_control_conditions_folded2_arm1(f,tx,ty,x(1));
            -drone_params_control_conditions_folded2_arm2(f,tx,ty,x(1));
            -drone_params_control_conditions_folded2_arm3(f,tx,ty,x(1));
            -drone_params_control_conditions_folded2_arm4(f,tx,ty,x(1));
        ];
    ceq = [];
end

end

