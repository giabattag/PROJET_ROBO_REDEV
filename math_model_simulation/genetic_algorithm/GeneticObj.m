classdef GeneticObj

properties
    type
    cost
    
    %weight terms (manual definitions)
    W = [150, 200, 0, 50];%[w_error, w_erroryaw, w_energy, w_overshot]
    %PID params
    K % [kp, kd, ki]
    Krange = 20;
    Pmutation = 0.1;
    
    fit %valeu that maximize to find the best fit
end


methods

function obj = GeneticObj(K, randType)
    if isempty(K(K~=0))
        obj.K = rand(size(K)).*obj.Krange;
    else
        obj.K = K;
        if randType == "norm"
            obj.K = (1+normrnd(0,obj.Pmutation)).*K;
        end
    end
   
end

function obj = cross_and_mutate(obj, parent1, parent2)
    
    K1 = parent1.K + (rand(1,size(obj.K,2))*obj.Pmutation - obj.Pmutation/2).*parent1.K;
    K2 = parent2.K + (rand(1,size(obj.K,2))*obj.Pmutation - obj.Pmutation/2).*parent2.K;
    
    obj.K = (K1 + K2)/2;

end

function obj = J(obj, error_p, error_yaw, command, over)
    integrate = obj.W(1).*error_p.^2 + obj.W(2).*error_yaw;% + obj.W(3).*sum(command,2).^2;
    obj.cost = trapz(integrate) + obj.W(4)*over;
%     obj.fit = 1/(energy + 1e-9);

end

function tf = lt(ob1, ob2)
    tf = ob1.cost < ob2.cost;
end

end
end