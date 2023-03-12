function [best,best_cost] = genetic_algo()

pop_size = 500;
keep_best = 30;
max_iter = 100;
max_iter_seq = 30;

population = cell(pop_size,1);
winners = cell(floor(pop_size/2)-keep_best);
for i = 1:pop_size
    population{i} = GeneticObj();
end
for i = 1:len(winners)
    population{i} = GeneticObj();
end
[best,population] = reorder(population,keep_best);

iters = 0;
iter_seq = 0;
best_cost = best.cost;

while iters < max_iter && iter_seq < max_iter_seq
    for winner=1:len(winners)
        [n1,n2] = differentRandom(0, pop_size-1);
        winners{winner} = min(population{n1}, population{n2});
    end

    population(keep_best:floor(pop_size/2)-1) = winners;

    for i = floor(pop_size/2):pop_size
        [n1,n2] = differentRandom(0, floor(pop_size/2)-1);
        population{i}.cross_and_mutate(population{n1}, population{n2});
    end

    best = reorder(population, keep_best);

    if best_cost > best.cost
        best_cost = best.cost;
        iter_seq = 0;
    else
        iter_seq = iter_seq + 1;
    end
    iters = iter_seq + 1;

end

%% FUNCTIONS
function [best,population] = reorder(population,keep_best)
%     population.sort()
%     return
%     for i in range(k):
%         idx = population[i:].index(min(population[i:]))
%         population[i],population[i+idx] = population[i+idx],population[i]

% return min(population[:keep_best])

    

end

function [n1,n2] = differentRandom(low,high)
    n1 = randi(low, high);
    n2 = randi(low, high-1);
    if n1 == n2
        if n1 < high
            n2 = n2+1;
        else
            n2 = n2-1;
        end
    end
end
end