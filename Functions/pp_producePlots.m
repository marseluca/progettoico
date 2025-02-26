function plots = pp_producePlots(trajectories,delta_s,flag)
    
    global nRobots pathColors;
    
    if flag
        
        figure(3)
        
        if nRobots>1
            minDistances = pp_getMinimumDistances(trajectories);

            [~, maxLengthIndex] = max(arrayfun(@(i) length(trajectories{i}.x_tot), 1:nRobots));
            maxTimeStep = length(minDistances);
            timeSteps = trajectories{maxLengthIndex}.t_tot(1:maxTimeStep);
            
            plot(timeSteps, minDistances,'-','LineWidth',1.2);
            xlabel('Time Step');
            ylabel('Minimum Distance');
            title('Minimum Distance at Each Time Step');
    
            hold on
            plot(timeSteps,delta_s*ones(1,length(minDistances)),'-','LineWidth',1.2);
            xlabel("t [s]")
            ylabel("$d(t)\:[m]$",'Interpreter','latex')
            title("Minimum distance between robots")
            legend("","Safety margin")
            grid
            hold off

        end


        for j=1:nRobots

            figure

            hold on
            subplot(2,2,1)
            plot(trajectories{j}.t_tot,trajectories{j}.x_tot,'-','LineWidth',1.2,'Color',pathColors(j,:));
            xlabel("t [s]")
            ylabel("$x(t)\:[m]$",'Interpreter','latex')
            grid on
            hold off
                
            hold on
            subplot(2,2,2)
            plot(trajectories{j}.t_tot,trajectories{j}.y_tot,'-','LineWidth',1.2,'Color',pathColors(j,:));
            xlabel("t [s]")
            ylabel("$y(t)\:[m]$",'Interpreter','latex')
            grid on
            hold off


            velocity_magnitude = sqrt(trajectories{j}.xdot_tot.^2 + trajectories{j}.ydot_tot.^2);
            acceleration_magnitude = sqrt(trajectories{j}.xddot_tot.^2 + trajectories{j}.yddot_tot.^2);
    
            hold on
            subplot(2, 2, 3)
            plot(trajectories{j}.t_tot,velocity_magnitude,'-','LineWidth',1.2,'Color',pathColors(j,:));
            xlabel("t [s]")
            ylabel("$|v(t)|\:[m]$",'Interpreter','latex')
            % ylim([0 1.5])
            grid on
            hold off
            
            hold on
            subplot(2, 2, 4)
            plot(trajectories{j}.t_tot,acceleration_magnitude,'-','LineWidth',1.2,'Color',pathColors(j,:));
            xlabel("t [s]")
            ylabel("$|a(t)|\:[m]$",'Interpreter','latex')
            % ylim([0 1])
            grid on
            hold off

            sgtitle("Robot "+j)
            
        end

    end
    
end

