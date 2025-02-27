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
animation = true;
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
    trajectories{j} = pp_interpolatePath2(paths{j},maxVelocity,0,0);
end

% Plotta i path interpolati sulla mappa
figure(1)
hold on
pp_plotPathOnMap(paths,trajectories,'-');

if solveCollisions
    ottimizzazione;

    trajectories = {};
    for j=1:nRobots
        trajectories{j} = pp_interpolatePath2(paths{j},x_opt(j),0,0);
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
