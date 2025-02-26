function plottedDistances = plotMinimumDistances(trajectories)
    
    global nRobots delta_s;

    figure
    hold on
    minDistances = pp_getMinimumDistances(trajectories);
    [~, maxLengthIndex] = max(arrayfun(@(i) length(trajectories{i}.x_tot), 1:nRobots));
    maxTimeStep = length(minDistances);
    timeSteps = trajectories{maxLengthIndex}.t_tot(1:maxTimeStep);
    plot(timeSteps, minDistances,'-','LineWidth',1.2);
    plot(timeSteps,delta_s*ones(1,length(minDistances)),'-','LineWidth',1.2);
    xlabel("t [s]")
    ylabel("$d(t)\:[m]$",'Interpreter','latex')
    title("Minimum distance between robots")
    legend("","Safety margin")
    hold off
    grid

end

