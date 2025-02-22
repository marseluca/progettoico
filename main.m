%% TRAJECTORY COMPUTATION

close all
clear all
global nRobots samplingTime pathColors maxVelocity maxAcceleration;
global paths delta_s;
global x_opt;

% Aggiungo la cartella Functions per utilizzare le funzioni al suo interno
addpath("Functions"); 

% Carico la mappa
openfig("MAP.fig");

% Definisco i path come set di punti nella mappa
paths = {};
paths{1} = [-90, 120; -90, 85; -15, 85; -15, -20; 40, -20; 40, -120];
paths{2} = [-75, 120; -75, 85; -15, 85; -15, -100; -100, -100];
paths{3} = [-120, 45; -15, 45; -15, -100; -100, -100];
paths{4} = [-120, -50; -80, -5; -75, 0; -15, 0; -15, 45; 10, 45; 42, 30; 75, 45; 100, 45; 100, -120];

nRobots = size(paths,2);

%% VARIABILI
robotSize = 6;
delta_s = 7;
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
end

%% INTERPOLAZIONE
trajectories = {};
for j=1:nRobots
    trajectories{j} = pp_interpolatePath2(paths{j},x_opt(j),0,0);
end


% %% COLLISION CHECKING
% collisions = {};
% for j=1:nRobots
%     collisions{j} = pp_checkCollisionForOneRobot(paths,trajectories,collisionThreshold,j);
% end

% Plotta le collisioni
% figure(1)
% hold on
% if plotCollisions
%     pp_plotCollisions(collisions,trajectories);
% end

% Crea i plot di posizione, velocità e accelerazione di ogni robot
% Crea anche il plot delle distanze minime tra i robot per ogni step
% temporale
if plotVelocities
    pp_producePlots(trajectories,delta_s,plotVelocities);
end


%% ANIMAZIONE
if animation
    fprintf("\nPress enter (or any key) to record animation with velocity %dx...\n",animVelocity);
    pp_animateTrajectory(trajectories,robotSize,recordAnimation,animVelocity);
end
