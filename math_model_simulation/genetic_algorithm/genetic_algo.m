function [best,best_cost] = genetic_algo()

pop_size = 10;
keep_best = 3;
max_iter = 100;
max_iter_seq = 30;

population = cell(pop_size,1);
winners = cell(floor(pop_size/2)-keep_best, 1);
%Generate all random childs
population{1} = GeneticObj([2, 1.5, 0]);
for i = 2:pop_size
    population{i} = GeneticObj(zeros(1,3));
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
    vs = population{1}.cost - population{10}.cost; 

    if best_cost > best.cost
        best_cost = best.cost;
        iter_seq = 0;
    else
        iter_seq = iter_seq + 1;
    end
    iters = iter_seq + 1;

end

disp('Best cost')



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
        global kp_position_pid kd_position_pid
 
        for i = 1:length(population)          
            kp_position_pid = population{i}.K(1);
            kd_position_pid = population{i}.K(2);
            simulation = sim('reconfig_dev_2021a_GB');
            err_z = (simulation.p.data(:,3) - simulation.pd.data(3));
            err_y = (simulation.p.data(:,2) - simulation.pd.data(2));
            err_x = (simulation.p.data(:,1) - simulation.pd.data(1));
            error = err_x + err_y + err_z;
            command = simulation.u.data(1:4);
            overshot_max = max([max(-sign(err_z(1))*err_z), max(-sign(err_y(1))*err_y), max(-sign(err_x(1))*err_x)]);
            population{i} = population{i}.J(error, command, overshot_max);
        end
       pop = population;
    end

end