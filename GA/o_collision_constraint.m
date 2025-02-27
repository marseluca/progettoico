function [c, ceq] = o_collision_constraint(x,paths,delta_s)
    
    global nRobots;
    c = [];
    ceq = [];

    for i = 1:nRobots
        trajectories{i} = pp_interpolatePath2(paths{i}, x(i), 0, 0);  
    end

    minDistances = pp_getMinimumDistances(trajectories);
    c = delta_s - min(minDistances);
    
end
