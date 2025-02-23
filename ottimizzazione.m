clc;

addpath("Functions"); 

% Numero di robot
global nRobots delta_s paths maxVelocity maxAcceleration;
global x_opt;

v_min = 0;
v_max = maxVelocity;

% Punto di partenza per l'ottimizzazione (velocità iniziali)
x0 = ones(nRobots,1) * v_max; % Moltiplicazione elemento per elemento

options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
                      'MaxIterations', 50000, ...
                      'MaxFunctionEvaluations', 200000, ...
                      'ConstraintTolerance', 1e-10, ...
                      'StepTolerance', 1e-12, ...
                      'OptimalityTolerance', 1e-10, ...
                      'Display', 'iter');


% Creazione del problema di ottimizzazione
problem = createOptimProblem('fmincon', 'objective', @(x) o_objective(x, paths), ...
                             'x0', x0, 'lb', v_min * ones(nRobots,1), ...
                             'ub', v_max * ones(nRobots,1), ...
                             'nonlcon', @(x) o_collision_constraint(x, paths, delta_s), ...
                             'options', options);


ms = MultiStart('UseParallel', true, 'Display', 'iter', 'StartPointsToRun', 'bounds-ineqs');
[x_opt, fval] = run(ms, problem, 500); % Prova con 500 punti iniziali


% Visualizzazione del risultato
disp('Velocità ottimali dei robot:');
disp(x_opt);
disp('Tempo totale minimo:');
disp(fval);

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

% Funzione obiettivo: minimizzare il tempo massimo tra i robot
function total_cost = o_objective(x, paths)
    global nRobots;
    total_travel_time = 0;

    for i = 1:nRobots
        slowedTraj = pp_interpolatePath2(paths{i}, x(i), 0, 0);  
        travel_time = slowedTraj.t_tot(end);
        total_travel_time = max(total_travel_time, travel_time);
    end
    
    total_cost = total_travel_time;
end
