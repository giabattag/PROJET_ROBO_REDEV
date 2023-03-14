classdef GeneticObj

properties
    type
    cost
    
    %weight terms (manual definitions)
    W = [100, 0.0001, 10];%[w_error, w_energy, w_overshot]
    %PID params
    K % [kp, kd, ki]
    Krange = 10;
    Pmutation = 7.5;
    
    fit %valeu that maximize to find the best fit
end


methods

function obj = GeneticObj(K)
    if length(K(K==0)) == 3
        obj.K = rand(1,3).*obj.Krange;
    else
        obj.K = K; 
    end
   
end

function obj = cross_and_mutate(obj, parent1, parent2)
    
    K1 = parent1.K + (rand(1,3)*obj.Pmutation - obj.Pmutation/2).*parent1.K;
    K2 = parent2.K + (rand(1,3)*obj.Pmutation - obj.Pmutation/2).*parent2.K;
    
    obj.K = (K1 + K2)/2;

end

function obj = J(obj, erro, u, over)
    integrate = obj.W(1).*erro.^2 + obj.W(2).*sum(u,2).^2;
    obj.cost = trapz(integrate) + obj.W(3)*over;
%     obj.fit = 1/(energy + 1e-9);

end

function tf = lt(ob1, ob2)
    tf = ob1.cost < ob2.cost;
end

end
end