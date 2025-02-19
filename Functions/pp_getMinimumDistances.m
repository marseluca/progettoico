function minDistances = pp_getMinimumDistances(trajectories)
    
    global nRobots;
    
    k = 1;

    %% APPROACH 1: SAVE TIME, LOTS OF MEMORY

    % maxLength = max(arrayfun(@(i) length(trajectories{i}.x_tot), 1:nRobots));
    % distances = Inf(maxLength, nRobots * (nRobots - 1) / 2); % Preallocate distances cell array
    % 
    % for i = 1:nRobots
    %     for j = i + 1:nRobots
    %         % Get all the distances over time
    %         maxLength = min(length(trajectories{i}.x_tot), length(trajectories{j}.x_tot));
    %         distance = sqrt((trajectories{i}.x_tot(1:maxLength) - trajectories{j}.x_tot(1:maxLength)).^2 + ...
    %                             (trajectories{i}.y_tot(1:maxLength) - trajectories{j}.y_tot(1:maxLength)).^2);
    %         distances(1:maxLength,k) = distance';
    %         k = k + 1;
    %     end
    % end
    % 
    % minDistances = min(distances, [], 2);
    % minDistances = minDistances';


    %% APPROACH 2: SAVE MEMORY, LOTS OF TIME
    % minDistances = Inf(1, maxLength); % Store only the minimum at each time step
    % for t = 1:maxLength
    %     for i = 1:nRobots
    %         for j = i+1:nRobots
    %             minLength = min(length(trajectories{i}.x_tot), length(trajectories{j}.x_tot));
    %             if t<=minLength
    %                 distance = sqrt((trajectories{i}.x_tot(t) - trajectories{j}.x_tot(t))^2 + ...
    %                                 (trajectories{i}.y_tot(t) - trajectories{j}.y_tot(t))^2);
    %                 minDistances(t) = min(minDistances(t), distance);
    %             end
    %         end
    %     end
    % end

    %% APPROACH 3: TRY TO SAVE BOTH
    % Preallocate only for pairs of robots
    nPairs = nRobots * (nRobots - 1) / 2;
    distances = cell(1, nPairs); % Use a cell array to avoid memory explosion
    
    % Calculate pairwise distances incrementally
    k = 1;
    for i = 1:nRobots
        for j = i+1:nRobots
            % Compute distances for the pair (i, j)
            maxLength = min(length(trajectories{i}.x_tot), length(trajectories{j}.x_tot));
            distances{k} = sqrt((trajectories{i}.x_tot(1:maxLength) - trajectories{j}.x_tot(1:maxLength)).^2 + ...
                                (trajectories{i}.y_tot(1:maxLength) - trajectories{j}.y_tot(1:maxLength)).^2);
            k = k + 1;
        end
    end
    
    % Calculate maximum number of time steps across all pairs
    maxLength = max(cellfun(@length, distances));
    nPairs = numel(distances);
    
    % Use block processing to minimize memory overhead
    minDistances = Inf(1, maxLength);
    
    for t = 1:maxLength
        minAtT = Inf;
        for k = 1:nPairs
            if t <= length(distances{k}) % Only process valid entries
                minAtT = min(minAtT, distances{k}(t));
            end
        end
        minDistances(t) = minAtT;
    end
    
end