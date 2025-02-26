% Funzione di vincolo per evitare collisioni
function [c, ceq] = o_collision_constraint(x, paths, delta_s)
    global nRobots;
    c = [];
    ceq = [];

    trajectories = cell(nRobots,1);
    for i = 1:nRobots
        trajectories{i} = pp_interpolatePath2(paths{i}, x(i), 0, 0);  
    end

    minDistances = pp_getMinimumDistances(trajectories);
    c = delta_s - min(minDistances); % Deve essere <= 0 per soddisfare il vincolo
end