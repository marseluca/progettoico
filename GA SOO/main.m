%% TRAJECTORY COMPUTATION

close all
clear all
global nRobots samplingTime pathColors maxVelocity maxAcceleration;
global paths delta_s;
global x_opt;

% Aggiungo la cartella Functions per utilizzare le funzioni al suo interno
addpath("C:\Users\Luca\Desktop\progettoico\Functions"); 

addpath("C:\Users\Luca\Desktop\progettoico\Functions\Interpolators"); 

% Carico la mappa
openfig("C:\Users\Luca\Desktop\progettoico\MAP.fig");

paths = load("C:\Users\Luca\Desktop\progettoico\paths.mat").paths;

nRobots = size(paths,2);

%% VARIABILI
robotSize = 5;
delta_s = 12;
collisionThreshold = delta_s;
maxVelocity = 10;
maxAcceleration = 20;

%% OPZIONI
animation = false;
animVelocity = 7;
recordAnimation = true;
solveCollisions = true;
preloadOptimization = false;
plotVelocities = false;
plotMinDistances = true;
plotCollisions = false;

samplingTime = 0.1;

% Assegna un colore casuale ad ogni path
pathColors = distinguishable_colors(nRobots);

%% INTERPOLAZIONE
trajectories = {};
for j=1:nRobots
    trajectories{j} = pp_interpolatePath2(paths{j},0,0);
end

% Plotta i path interpolati sulla mappa
figure(1)
hold on
pp_plotPathOnMap(paths,trajectories,'-');

%% COLLISIONS
tic
if solveCollisions

    if preloadOptimization==false
    
    global x_opt;

    % Set ub and lb
    lb = [];
    ub = [];

    for i = 1:nRobots
        % Lower bounds for d_i, L_s, and alpha
        lb = [lb, 0, 0, 0.1];

        % Calculate the upper bound for L_s based on segments
        max_Ls = norm(paths{i}(2,:) - paths{i}(1,:)); % Length of the first segment

        % Upper bounds for d_i (binary), L_s (max segment length), and alpha
        ub = [ub, 1, max_Ls, 1];
    end

    % Set the integer constraints for d_i
    intcon = 1:3:(3*nRobots); % Indices for d_i

    % Set optimization options
    % Set the options for the genetic algorithm
    options_ga = optimoptions('ga', ...
    'Display', 'iter', ...
    'MaxGenerations', 500, ...
    'PopulationSize', 100, ...
    'CrossoverFraction', 0.9, ...
    'MutationFcn', {@mutationuniform, 0.1},...
    'PlotFcn', [], ... % Use a different plot function suitable for multi-objective
    'MaxStallGenerations', 1000, ...
    'OutputFcn', @o_outputFcn2); % If you want to use a similar concept for multi-objective

    % Call the ga solver

    objectiveFun = @(x) o_objective(x,paths);
    constraintFun = @(x) o_collision_constraint(x,paths,delta_s);
    
    tic
    [x_opt, fval] = ga(objectiveFun, 3*nRobots, [], [], [], [], lb, ub, constraintFun, intcon, options_ga);
    executionTime = toc;
    
    else
        x_opt = [0	22.5452708212247	0.722279072113670	1	27.9344993920384	0.205097102420925	0	70.8050395187980	0.789094047499266	1	50.9832296086777	0.379036197787235];
    end
end

finishTimes = [];
for j=1:nRobots
    finishTimes = [finishTimes, trajectories{j}.t_tot(end)];
end
fprintf("Finish time: %.2f",max(finishTimes))

%% COLLISION CHECKING
collisions = {};
for j=1:nRobots
    collisions{j} = pp_checkCollisionForOneRobot(paths,trajectories,collisionThreshold,j);
end

figure(1)
hold on
if plotCollisions
    pp_plotCollisions(collisions,trajectories);
end

if plotVelocities
    pp_producePlots(trajectories,delta_s,plotVelocities);
end

if plotMinDistances
    plotMinimumDistances(trajectories);
end

%% ANIMAZIONE
if animation
    fprintf("\nPress enter (or any key) to record animation with velocity %dx...\n",animVelocity);
    pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity);
end
