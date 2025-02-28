function [c, ceq] = o_collision_constraint(x,paths,delta_s)
    
    global nRobots;
    c = [];
    ceq = [];


    for i = 1:nRobots
        d_i = x(3*(i-1) + 1); % Decision variable for robot i
        L_s = x(3*(i-1) + 2); % Length of slow-down segment for robot i
        alpha = x(3*(i-1) + 3); % Velocity scaling factor for robot i

        % Get the path for robot i
        path = paths{i};
        if d_i == 1
            newSegment = 1;
            path = pp_addNewSegment(path, newSegment, 0, L_s);
            path = unique(path, 'rows', 'stable');
            trajectories{i} = pp_interpolatePath2(path, newSegment, alpha);
        else
            trajectories{i} = pp_interpolatePath2(path, 0, 0);
        end        
    end

    minDistances = pp_getMinimumDistances(trajectories);
    c = delta_s - min(minDistances);


end