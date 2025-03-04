function trajectory = pp_interpolatePath2(path,vmax,segmentToIncrease,scalingFactor)
    
    global maxAcceleration;
    
    amax = maxAcceleration;
    numberOfSamples = 100;
    interpolatedPoints = {};
    trajectory = struct();


    tolerance = 1e-20; % Define a small tolerance
    path = unique(round(path / tolerance) * tolerance, 'rows', 'stable');

    x = path(:,1);
    y = path(:,2);

    % Add a point at the beginning of the path
    % Between the first and the second point
    % This allows the initial velocity to be zero

     % Calculate the midpoint coordinates
    x_mid = (x(1) + x(2)) / 2;
    y_mid = (y(1) + y(2)) / 2;

    x = [x_mid; x];
    y = [y_mid; y];

    x_mid = (x(end-1) + x(end)) / 2;
    y_mid = (y(end-1) + y(end)) / 2;

    x = [x; x_mid];
    y = [y; y_mid];

    segment = 1;

    lengths = sqrt(diff(x).^2 + diff(y).^2);
    segment_times = lengths / vmax;

    % Increase segment time
    if segmentToIncrease~=0
        segmentToIncrease = segmentToIncrease+1; % Because you add 1 segment 
                                                 % at the beginning
                                                 % That you will remove
                                                 % because you need it to
                                                 % set the initial velocity
                                                 % of the robot to zero
        segment_times(segmentToIncrease) = lengths(segmentToIncrease) / (scalingFactor*vmax);
    end
    
    t = [0, cumsum(segment_times)'];

    spline_x = pchip(t,x);
    spline_y = pchip(t,y);

    trajectory.t_tot = linspace(t(1), t(end), numberOfSamples);

    % Evaluate the splines
    trajectory.x_tot = ppuval(trajectory.t_tot,spline_x);
    trajectory.y_tot = ppuval(trajectory.t_tot,spline_y);   


    % === Compute the velocity ===
    % Differentiate the pchip splines to get velocity
    velocity_x_spline = pp_derivateSplineVel(spline_x);
    velocity_y_spline = pp_derivateSplineVel(spline_y);
    
    % Evaluate the velocity components at the time points
    trajectory.xdot_tot = ppuval(trajectory.t_tot,velocity_x_spline);
    trajectory.ydot_tot = ppuval(trajectory.t_tot,velocity_y_spline);

    %% Remove the first and the last segment
    x_val = trajectory.x_tot;
    y_val = trajectory.y_tot;
    distancesFromFirstPoint = sum(([x_val',y_val']-[x(2),y(2)]).^2,2);
    distancesFromLastPoint = sum(([x_val',y_val']-[x(end-1),y(end-1)]).^2,2);

    % Find the index of the minimum distance
    [~, firstPointIndex] = min(distancesFromFirstPoint);
    [~, lastPointIndex] = min(distancesFromLastPoint);
    %%

    % Compute the velocity magnitude at each time step
    velocity_magnitude = sqrt(trajectory.xdot_tot(firstPointIndex:lastPointIndex).^2 + trajectory.ydot_tot(firstPointIndex:lastPointIndex).^2);
    
    % Find the maximum velocity encountered
    max_encountered_velocity = max(velocity_magnitude);

    % === Compute the acceleration ===
    % Differentiate the velocity splines to get acceleration
    acceleration_x_spline = pp_derivateSplineAcc(velocity_x_spline); % Second derivative of position (x)
    acceleration_y_spline = pp_derivateSplineAcc(velocity_y_spline); % Second derivative of position (y)
    
    % Evaluate the acceleration components at the time points
    trajectory.xddot_tot = ppuval(trajectory.t_tot,acceleration_x_spline);
    trajectory.yddot_tot = ppuval(trajectory.t_tot,acceleration_y_spline);


    % Compute the acc magnitude at each time step
    acceleration_magnitude = sqrt(trajectory.xddot_tot(firstPointIndex:lastPointIndex).^2 + trajectory.yddot_tot(firstPointIndex:lastPointIndex).^2);
    
    % Find the maximum acc encountered
    max_encountered_acceleration = max(acceleration_magnitude);
    
    % Check if the encountered velocity exceeds the max allowed velocity
    if max_encountered_velocity > vmax || max_encountered_acceleration > amax
        % Scale the maximum velocity down to avoid collisions

        if max_encountered_velocity > vmax && max_encountered_acceleration <= amax
            scaling_factor = vmax / max_encountered_velocity;
        elseif max_encountered_acceleration > amax && max_encountered_velocity <= vmax
            scaling_factor = amax / max_encountered_acceleration;
        else
            scaling_factor1 = vmax / max_encountered_velocity;
            scaling_factor2 = amax / max_encountered_acceleration;
            scaling_factor = min(scaling_factor1,scaling_factor2);
        end

        vmax = scaling_factor*vmax;

        segment = 1;

        lengths = sqrt(diff(x).^2 + diff(y).^2);
        segment_times = lengths / vmax;

        % Increase segment time
        if segmentToIncrease~=0
            segment_times(segmentToIncrease) = lengths(segmentToIncrease) / (scalingFactor*vmax);
        end

        
        t = [0, cumsum(segment_times)'];

        spline_x = pchip(t,x);
        spline_y = pchip(t,y);

        trajectory.t_tot = linspace(t(1), t(end), numberOfSamples);

        % Evaluate the splines
        trajectory.x_tot = ppuval(trajectory.t_tot,spline_x);
        trajectory.y_tot = ppuval(trajectory.t_tot,spline_y); 

        % === Compute the velocity ===
        % Differentiate the pchip splines to get velocity
        velocity_x_spline = pp_derivateSplineVel(spline_x);
        velocity_y_spline = pp_derivateSplineVel(spline_y);
       

        % Evaluate the velocity components at the time points
        trajectory.xdot_tot = ppuval(trajectory.t_tot,velocity_x_spline);
        trajectory.ydot_tot = ppuval(trajectory.t_tot,velocity_y_spline);


        % === Compute the acceleration ===
        % Differentiate the velocity splines to get acceleration
        acceleration_x_spline = pp_derivateSplineAcc(velocity_x_spline); % Second derivative of position (x)
        acceleration_y_spline = pp_derivateSplineAcc(velocity_y_spline); % Second derivative of position (y)
        
        % Evaluate the acceleration components at the time points
        trajectory.xddot_tot = ppuval(trajectory.t_tot,acceleration_x_spline);
        trajectory.yddot_tot = ppuval(trajectory.t_tot,acceleration_y_spline);
    end
   
    
    %% Remove the first and last segment
    trajectory.x_tot = trajectory.x_tot(firstPointIndex:lastPointIndex);
    trajectory.y_tot = trajectory.y_tot(firstPointIndex:lastPointIndex);
    trajectory.xdot_tot = trajectory.xdot_tot(firstPointIndex:lastPointIndex);
    trajectory.ydot_tot = trajectory.ydot_tot(firstPointIndex:lastPointIndex);
    trajectory.xddot_tot = trajectory.xddot_tot(firstPointIndex:lastPointIndex);
    trajectory.yddot_tot = trajectory.yddot_tot(firstPointIndex:lastPointIndex);
    trajectory.t_tot = trajectory.t_tot(firstPointIndex:lastPointIndex);
    %%

    trajectory.t_tot = trajectory.t_tot - trajectory.t_tot(1);
    %%



    % % Common sample time
    trajectory = pp_commonTimeSampling(trajectory);


    
end

