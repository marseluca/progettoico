clc;

addpath("C:\Users\Luca\Desktop\progettoico\Functions"); 
addpath("C:\Users\Luca\Desktop\progettoico\Optim Functions"); 

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
                      'Display', 'none');

lb = v_min * ones(nRobots,1);
ub = v_max * ones(nRobots,1);

% Creazione del problema di ottimizzazione
problem = createOptimProblem('fmincon', 'objective', @(x) o_objective(x, paths), ...
                             'x0', x0, 'lb', lb, 'ub', ub, ...
                             'nonlcon', @(x) o_collision_constraint(x, paths, delta_s), ...
                             'options', options);


ms = MultiStart('UseParallel', true, 'Display', 'none', 'StartPointsToRun', 'bounds-ineqs');
[x_opt, fval] = run(ms, problem, 500);

% gs = GlobalSearch('Display', 'final', 'StartPointsToRun', 'bounds-ineqs');
% [x_opt, fval] = run(gs, problem);


% Visualizzazione del risultato
disp('Velocità ottimali dei robot:');
disp(x_opt);
disp('Tempo totale minimo:');
disp(fval);
