function [x_opt,fval] = runOptimization(paths,delta_s,maxGen,popSize,CF,MR)
 
    global nRobots maxVelocity;

    %% Upper e lower bound
    lb = [];
    ub = [];

    for i = 1:nRobots
        % Lower bounds for d_i, L_s, and alpha
        lb = [lb, 0.1];

        % Upper bounds for d_i (binary), L_s (max segment length), and alpha
        ub = [ub, maxVelocity];
    end

    %% Opzioni ottimizzazione
    options_ga = optimoptions('ga', ...
    'Display', 'final', ...
    'EliteCount',1, ...
    'MaxGenerations', maxGen, ...
    'PopulationSize', popSize, ...
    'CrossoverFraction', CF, ...
    'MutationFcn', {@mutationuniform, MR},...
    'PlotFcn', [], ...
    'MaxStallGenerations', 1000, ...
    'OutputFcn', @o_outputFunction);

    
    %% Solver
    objectiveFun = @(x) o_objective(x,paths);
    constraintFun = @(x) o_collision_constraint(x,paths,delta_s);
    
    [x_opt, fval] = ga(objectiveFun, nRobots, [], [], [], [], lb, ub, constraintFun, options_ga);
    
end

