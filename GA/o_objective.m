function total_cost = o_objective(x,paths)

    global nRobots;

    total_travel_time = 0;

    for i = 1:nRobots

        slowedTraj = pp_interpolatePath2(paths{i}, x(i), 0, 0);  
        
        travel_time = slowedTraj.t_tot(end);

        total_travel_time = max(total_travel_time, travel_time);

    end
    
    total_cost = total_travel_time;

end