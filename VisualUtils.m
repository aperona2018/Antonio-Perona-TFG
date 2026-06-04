classdef VisualUtils

    methods (Static)
        function animatePath(app, startConfig, interpPath, segs)
            appName = class(app);
        
            ax = findobj(app.SimulationPanel, 'Type', 'axes');
            if isempty(ax)
                ax = uiaxes(app.SimulationPanel);
            else
                ax = ax(1);  
            end
        
            hold(ax, 'on');
            axis(ax, 'equal');
            grid(ax, 'on');
            xlabel(ax, 'Z'); ylabel(ax, 'X'); zlabel(ax, 'Y');
            view(ax, 3);

            xlim(ax, [-2 2]); 
            ylim(ax, [-2 2]);
            zlim(ax, [-2 2]);
        
            for k = 1:numel(app.env)
                show(app.env{k}, 'Parent', ax);
                drawnow limitrate;
            end
        
            for i = 1:size(interpPath, 1)
                config_step = interpPath(i, :);  
        
                config = startConfig;  
                for j = 1:length(config)
                    config(j).JointPosition = config_step(j);
                end
        
                show(app.robot, config, 'PreservePlot', false, 'Parent', ax, 'Collisions', 'on', 'FastUpdate', true);
                drawnow limitrate;

                app.robotCurrentConfig = config;

                if (app.operationMode == "Simulation") && (appName == "app_robot_control")
                    app.updateDegreesCurrentConfig(app.robotCurrentConfig);
                    app.updateCartesianCurrentConfig(app.robotCurrentConfig);
                end
                pause(segs);
            end
            hold off;
        end

        function showCollisionBox(obj, ax)

            [V, F] = collisionMesh(obj);
        
            patch('Vertices', V, ...
                  'Faces', F, ...
                  'FaceColor', [0.6 0.6 0.6], ...
                  'FaceAlpha', 0.3, ...
                  'EdgeColor', 'none', ...
                  'Parent', ax);
        
        end

        function showRobotEnvPanel(SimulationPanel, AppRobotInstance, robotCurrentConfig, env)
            delete(findall(SimulationPanel, 'Type', 'axes'));
            ax = axes('Parent', SimulationPanel);  % Estilo explícito
            
            hold(ax, 'on');
            axis(ax, 'equal');
            grid(ax, 'on');
            xlabel(ax, 'Z'); ylabel(ax, 'X'); zlabel(ax, 'Y');
            view(ax, 3);
            
            show(AppRobotInstance, robotCurrentConfig, 'Parent', ax, 'Collisions', 'on');
            for i = 1:length(env)
                show(env{i}, 'Parent', ax);
            end
            
            margin = 1.5;
            xlim(ax, [-margin, margin]);
            ylim(ax, [-margin, margin]);
            zlim(ax, [-margin, margin]);
            
            hold(ax, 'off');                     
        end
    end
end