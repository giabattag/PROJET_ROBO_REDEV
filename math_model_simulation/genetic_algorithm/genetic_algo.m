% function [best,best_cost] = genetic_algo()

main

global model kp_z_stw kd_z_stw kp_x_stw kd_x_stw kp_y_stw kd_y_stw k11_attitude_stw k12_attitude_stw k13_attitude_stw ...
    k21_attitude_stw k22_attitude_stw k23_attitude_stw tuning_parameter_stw ki_z_stw ki_x_stw ki_y_stw ...
    kp_position_pid kd_position_pid ki_position_pid kp_attitude_pid kd_attitude_pid

model = 'reconfig_dev';

k_init = [kp_x_stw, kd_x_stw, ki_x_stw, kp_y_stw, kd_y_stw, ki_y_stw, kp_z_stw, kd_z_stw, ki_z_stw,...
            k11_attitude_stw k12_attitude_stw k13_attitude_stw,...
            k21_attitude_stw k22_attitude_stw k23_attitude_stw];

% k_init = [kp_position_pid kd_position_pid ki_position_pid kp_attitude_pid kd_attitude_pid];

pop_size = 30;
keep_best = 10;
max_iter = 100;
max_iter_seq = 10;
gen = -10;

population = cell(pop_size,1);
winners = cell(floor(pop_size/2)-keep_best, 1);
%Generate all random childs
population{1} = GeneticObj(k_init, ""); %for known possible seed
for i = 2:pop_size/2-1
    population{i} = GeneticObj(k_init, "norm");
end
for i = pop_size/2:pop_size
    population{i} = GeneticObj(zeros(size(k_init)));
end

% for i = 2:length(winners)
%     population{i} = GeneticObj(zeros(1,3));
% end

%run the simulation enviroment to get results
population = simulation_env(population);

%rank the generation
[best,population] = reorder(population,keep_best);

iters = 0;
iter_seq = 0;
best_cost = best.cost;

while iters < max_iter && iter_seq < max_iter_seq
    for winner=1:length(winners)
        [n1,n2] = differentRandom(1, pop_size-1);
        if population{n1} < population{n2} 
            winners{winner} = population{n1};
        else
            winners{winner} = population{n2};
        end
    end

    population(keep_best:floor(pop_size/2)-1) = winners;

    for i = floor(pop_size/2):pop_size
        [n1,n2] = differentRandom(1, floor(pop_size/2)-1);
        population{i} = population{i}.cross_and_mutate(population{n1}, population{n2});
    end
    
    population = simulation_env(population);
    [best, population] = reorder(population, keep_best);

    if best_cost > best.cost
        best_cost = best.cost;
        iter_seq = 0;
        if gen < iters+10
            gen = iters;
            generation_result(best, gen);
        end
        
    else
        iter_seq = iter_seq + 1;
    end
    iters = iters + 1;

end

generation_result(best, max_iter+1);


%% FUNCTIONS
function [best,population] = reorder(population,keep_best)
%     population.sort()
%     return
%     for i in range(k):
%         idx = population[i:].index(min(population[i:]))
%         population[i],population[i+idx] = population[i+idx],population[i]

% return min(population[:keep_best])

    costs_array = cellfun(@(x)x.cost, population);

    [~, index_sorted] = sort(costs_array);
    population = population(index_sorted);
%     population = population(1:keep_best);
    best = population{1};

    

end

function [n1,n2] = differentRandom(low,high)
    n1 = randi([low, high]);
    n2 = randi([low, high-1]);
    if n1 == n2
        if n1 < high
            n2 = n2+1;
        else
            n2 = n2-1;
        end
    end
end

    function  pop = simulation_env(population)
        global model kp_z_stw kd_z_stw kp_x_stw kd_x_stw kp_y_stw kd_y_stw k11_attitude_stw k12_attitude_stw k13_attitude_stw ...
            k21_attitude_stw k22_attitude_stw k23_attitude_stw tuning_parameter_stw ki_z_stw ki_x_stw ki_y_stw ...
            kp_position_pid kd_position_pid ki_position_pid kp_attitude_pid kd_attitude_pid
 
        for i = 1:length(population)
            kp_x_stw = population{i}.K(1);
            kd_x_stw = population{i}.K(2);
            ki_x_stw = population{i}.K(3);
            kp_y_stw = population{i}.K(4);
            kd_y_stw = population{i}.K(5);
            ki_y_stw = population{i}.K(6);
            kp_z_stw = population{i}.K(7);
            kd_z_stw = population{i}.K(8);
            ki_z_stw = population{i}.K(9);
            k11_attitude_stw = population{i}.K(10);
            k12_attitude_stw = population{i}.K(11);
            k13_attitude_stw = population{i}.K(12);
            k21_attitude_stw = population{i}.K(13);
            k22_attitude_stw = population{i}.K(14);
            k23_attitude_stw = population{i}.K(15);
%             kp_position_pid = population{i}.K(1);
%             kd_position_pid = population{i}.K(2);
%             ki_position_pid = population{i}.K(3);
%             kp_attitude_pid = population{i}.K(4);
%             kd_attitude_pid = population{i}.K(5);
            simulation = sim(model);
            err_z = simulation.err_position(:,3);
            err_y = simulation.err_position(:,2);
            err_x = simulation.err_position(:,1);
            error = abs(err_x) + abs(err_y) + abs(err_z);
            command = simulation.prop_com;
            err_yaw = simulation.err_yaw;
            overshot_max = max([max(err_z(err_z>0)), max(err_y(err_y>0)), max(err_x(err_x>0))]);
            population{i} = population{i}.J(error, err_yaw, command, overshot_max);
        end
       pop = population;
    end

    function generation_result(best, gen)
%         f = figure();
%         hold on; grid on;
        global model kp_z_stw kd_z_stw kp_x_stw kd_x_stw kp_y_stw kd_y_stw ki_z_stw ki_x_stw ki_y_stw ...
            k11_attitude_stw k12_attitude_stw k13_attitude_stw ...
            k21_attitude_stw k22_attitude_stw k23_attitude_stw tuning_parameter_stw ...
            kp_position_pid kd_position_pid ki_position_pid kp_attitude_pid kd_attitude_pid
        kp_x_stw = best.K(1);
        kd_x_stw = best.K(2);
        ki_x_stw = best.K(3);
        kp_y_stw = best.K(4);
        kd_y_stw = best.K(5);
        ki_y_stw = best.K(6);
        kp_z_stw = best.K(7);
        kd_z_stw = best.K(8);
        ki_z_stw = best.K(9);
        k11_attitude_stw = best.K(10);
        k12_attitude_stw = best.K(11);
        k13_attitude_stw = best.K(12);
        k21_attitude_stw = best.K(13);
        k22_attitude_stw = best.K(14);
        k23_attitude_stw = best.K(15);
%         kp_position_pid = best.K(1);
%         kd_position_pid = best.K(2);
%         ki_position_pid = best.K(3);
%         kd_attitude_pid = best.K(4);
%         kp_attitude_pid = best.K(5);
%         sim(model);
        
%         plot(simulation.err_position);
%         legend('x', 'y', 'z');
%         dim = [0.2 0.5 0.3 0.3];
%         str = {string(best.K)};
%         savefig(f, string(kp_z_stw)+'_'+string(kd_z_stw)+'_'+string(ki_z_stw)+'_'+string(kp_x_stw)+'_'+...
%             string(kd_x_stw)+'_'+ string(ki_x_stw)+'_'+ string(kp_y_stw)+'_'+...
%             string(kd_y_stw)+'_'+ string(ki_y_stw)+'_'+'best_gen' + string(gen) +'.fig');
%         savefig(f, string(kp_position_pid)+'_'+string(kd_position_pid)+'_'+string(ki_position_pid)+'_'+string(kd_attitude_pid)+'_'+...
%             string(kp_attitude_pid)+'_'+'best_gen' + string(gen) +'.fig');
%         close all;
        save("stw_best_gen_"+num2str(gen)+".mat","best");
    
    end

% end