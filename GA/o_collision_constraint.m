function [c, ceq] = o_collision_constraint(x,paths,delta_s)
    
    global nRobots;
    c = [];
    ceq = [];

    for j = 1:nRobots
        trajectories{j} = pp_interpolatePath2(paths{j},x(j),0,0);        
    end

    minDistances = pp_getMinimumDistances(trajectories);
    c = delta_s - min(minDistances);

end