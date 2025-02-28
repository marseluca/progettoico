%% TRAJECTORY COMPUTATION

close all
clear all


%% Caricamenti
addpath("C:\Users\Luca\Desktop\progettoico\Functions"); 
addpath("C:\Users\Luca\Desktop\progettoico\Functions\Interpolators"); 
openfig("C:\Users\Luca\Desktop\progettoico\MAP.fig");
paths = load("C:\Users\Luca\Desktop\progettoico\paths.mat").paths;
nRobots = size(paths,2);

%% Variabili globali
global nRobots samplingTime pathColors maxVelocity maxAcceleration;
global paths delta_s;
global x_opt;

%% Variabili locali
robotSize = 5;
delta_s = 12;
collisionThreshold = delta_s;
maxVelocity = 10;
maxAcceleration = 20;
samplingTime = 0.1;
pathColors = distinguishable_colors(nRobots);

%% OPZIONI
animation = false;
animVelocity = 7;
recordAnimation = true;
solveCollisions = true;
preloadOptimization = false;
plotVelocities = true;
plotMinDistances = true;
plotCollisions = false;


%% Interpolazione iniziale
trajectories = {};
for j=1:nRobots
    trajectories{j} = pp_interpolatePath2(paths{j},maxVelocity,0,0);
end

% Plotta i path interpolati sulla mappa
figure(1)
hold on
pp_plotPathOnMap(paths,trajectories,'-');

%% COLLISION CHECKING
collisions = {};
for j=1:nRobots
    collisions{j} = pp_checkCollisionForOneRobot(paths,trajectories,collisionThreshold,j);
end

%% Ottimizzazione
if solveCollisions

    if preloadOptimization==false
        % Parametri: paths, max generations, population size, CF, MR
        [x_opt,fval] = runOptimization(paths,delta_s,500,1000,0.6,0.1);
    else
        x_opt = [8.6009    3.1809    9.0477    4.6073];
    end

    trajectories = {};
    for j=1:nRobots
        trajectories{j} = pp_interpolatePath2(paths{j},x_opt(j),0,0);
    end
end


%% Plot

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

%% Animazione
if animation
    fprintf("\nPress enter (or any key) to record animation with velocity %dx...\n",animVelocity);
    pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity);
end
