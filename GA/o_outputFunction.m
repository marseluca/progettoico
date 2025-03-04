function [state, options, optchanged] = o_outputFunction(options, state, flag)
    optchanged = false;  % Non so a cosa serva
    global maxGen;
    global paths delta_s;
    global validSim bestFitVector bestSolution;

    % Ottieni la generazione attuale
    gen = state.Generation + 1;  % Aggiungi 1 altrimenti inizierebbe da zero

    if gen<=1
        bestFitVector = [];
    end

    fprintf("\nGenerazione: %d", gen);


    % Calcola la constraint violation per tutte le soluzioni
    constraintViolations = zeros(size(state.Population, 1), 1);
    for i = 1:size(state.Population, 1)
        x_candidate = state.Population(i, :);
        [cineq, ~] = o_collision_constraint(x_candidate, paths, delta_s);
        constraintViolations(i) = max(cineq);
    end
    
    %% Salva solo le soluzioni idonee

    % Filtra solo le soluzioni con max constraint violation negativa
    validIndices = find(constraintViolations < 0);
    if isempty(validIndices)
        fprintf(" - Nessuna soluzione valida trovata nella generazione %d\n", gen);
    else
        validConstraints = constraintViolations(validIndices);
        validSolutions = state.Population(validIndices, :);

        % Elitism
        if gen>1
            worstFit = -Inf;
            for i=1:length(validSolutions)
                currentFit = o_objective(validSolutions(i,:),paths);
                if currentFit > worstFit
                    worstFit = currentFit;
                    worstFitIndex = i;
                end
            end
            validSolutions(worstFitIndex, :) = bestSolution;
        end

        % Trova il miglior fit positivo
        bestFit = Inf;
        for i=1:length(validSolutions)
            currentFit = o_objective(validSolutions(i,:),paths);
            if currentFit < bestFit
                bestFit = currentFit;
                bestFitIndex = i;
            end
        end
        
        constraintViolation = constraintViolations(bestFitIndex);    
        bestSolution = validSolutions(bestFitIndex,:);
        fprintf(" - Best positive fit: %.2f, Constraint Violation: %.2f\n", bestFit, validConstraints(i));
    
        bestFitVector = [bestFitVector, bestFit];
    end


    if isempty(bestFitVector)
    else
        figure(2)
        plot(bestFitVector,'b-','LineWidth',1.2)
        grid
        ylim([0 Inf])
    end
    
    % Stop execution after maxGen generations
    if gen >= maxGen
        state.StopFlag = 'y';  % Stop execution
    end
end