function [state, options, optchanged] = o_outputFcn2(options, state, flag)
    optchanged = false;  % Default is no change to options
    global maxGen;
    global paths delta_s;
    global feasibleSolution validSim;

    % Get the current generation and best fitness
    gen = state.Generation + 1;  % Add 1 so gen starts from 1 instead of 0

    % Stop execution after maxGen generations
    if gen >= maxGen

        [~, idx] = min(state.Score); 
        x_opt = state.Population(idx, :); % Best solution so far (x_opt)
    
        % Evaluate constraints for the best solution
        [cineq, ~] = o_collision_constraint(x_opt,paths,delta_s);  % Replace with your constraint function name
        maxConstraintViolation = max(cineq); % Max constraint violation

        if maxConstraintViolation<=0
            validSim = true;
            feasibleSolution = x_opt;
        else
            validSim = false;
        end

        state.StopFlag = 'y';  % Stop execution
    end
end
