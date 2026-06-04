classdef MovementUtils

    methods (Static)
        function planner = initializePlanner(robot, env, validationDist)
            planner = manipulatorRRT(robot, env);
            planner.SkippedSelfCollisions = 'parent';
            planner.ValidationDistance = validationDist;
        end

        function [interpPath, status] = resolveMovementDirectDegrees(planner, ...
                startVector, goalVector, UIFigure)
            try
                path = plan(planner, startVector, goalVector);
                interpPath = interpolate(planner, path);

                if isempty(interpPath)
                    status = -1;
                    interpPath = [];
                    uialert(UIFigure, "Interpolation failed: empty path.", "Interpolation Error");
                    return;
                end

                status = 0;
            catch ME
                status = -1;
                interpPath = [];
                switch ME.identifier
                    case 'Robotics:Planner:Collision'
                        msg = sprintf("ERROR: The goal configuration is in collision");
                        uialert(UIFigure, msg, "Goal configuration in collision");
                        return;
                    case 'InvalidConfiguration:OutOfBounds'                    
                        msg = sprintf("ERROR: The goal configuration is out of bounds");
                        uialert(UIFigure, msg, "Goal configuration out of bounds");                     
                        return;
                    otherwise
                        uialert(UIFigure, ME.message, ME.identifier);                      
                        return;
                end
            end
        end

        function targetPose = defineIkPose(app, angles_rot_y, t_x, t_y, ...
                t_z)
            mat_transl = transl(t_x, t_y, t_z);
            if (strcmp(app.movementMode, "Movej"))
                targetPose = mat_transl * roty(angles_rot_y) * rotz(pi/2);
            else
                targetPose = rotz(pi) * mat_transl * roty(angles_rot_y) * rotz(pi/2);
            end
        end

        function [goalConfig, status] = resolveInverseKinematics(robot, targetPose, weights, robotCurrentConfig, UIFigure, toolName)
            ik = inverseKinematics('RigidBodyTree', robot);
            [goalConfig, solInfo] = ik(toolName, targetPose, weights, robotCurrentConfig);
            status = 0;

            if ~strcmp(solInfo.Status, 'success') && ~strcmp(solInfo.Status, 'best available')
                msg = sprintf('ERROR: Could not find a valid solution');
                uialert(UIFigure, msg, "Solution not found") 
                status = -1;
                return;
            end
        end

        function [status] = moveRTDERobotJoints(robotRTDEClient, robotPort, ...
                goalConfigVector, toleranceErrorDegrees, acc, speed, app)
            
            validPosition = false;

            q = goalConfigVector;


            cmd = sprintf('movej([%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], a=%.4f, v=%.4f)\n', ...
                q(1), q(2), q(3), q(4), q(5), q(6), acc, speed);
            try
                executeURScriptCommand(robotRTDEClient, cmd, robotPort);

                actualConfig(6) = struct('JointPosition', 0);
                while (~validPosition)
                    actualPosition = readJointConfiguration(robotRTDEClient);

                    for i = 1:6
                        actualConfig(i).JointPosition = actualPosition(i);
                    end
                    if (strcmp(class(app), "app_robot_control"))     
                        app.updateDegreesCurrentConfig(actualConfig);
                        app.updateCartesianCurrentConfig(actualConfig);
                        if (app.toggleAnimation)
                            VisualUtils.showRobotEnvPanel(app.SimulationPanel, app.robot, app.robotCurrentConfig, app.env);
                        end
                    end
            
                    positionError = abs(actualPosition - q);
            
                    if all(positionError < deg2rad(toleranceErrorDegrees))
                        validPosition = true;
                        pause(0.1);
                    end
                end
                status = 0;
            catch ME
                status = -1;
                uialert(app.UIFigure, ME.message, "Joints movement error");
                return;
            end
        end

        function [status] = moveRTDERobotLinear(robotRTDEClient, robotPort, ...
            targetPose, toleranceErrorMeters, acc, speed, app)
            
            validPosition = false;
        
            pos = tform2trvec(targetPose);
            rot = tform2rotm(targetPose);
            axang = rotm2axang(rot);
        
            rotvec = axang(1:3) * axang(4);
        
            pose = [pos rotvec];
        
            cmd = sprintf('movel(p[%.4f, %.4f, %.4f, %.4f, %.4f, %.4f], a=%.4f, v=%.4f)\n', ...
                pose(1), pose(2), pose(3), pose(4), pose(5), pose(6), acc, speed);
            try
                executeURScriptCommand(robotRTDEClient, cmd, robotPort);
        

                while (~validPosition)
        
                    if strcmp(class(app),"app_robot_control")
        
                        actualPosition = readJointConfiguration(robotRTDEClient);

                        for i = 1:6
                            actualConfig(i).JointPosition = actualPosition(i);
                        end
                        robotCurrentConfig = Utils.makeValidConfig(app.robot, actualConfig);

                        tform = getTransform(app.robot, robotCurrentConfig, 'tool0');
                        actualPose = tform(1:3,4);

                        actualPose_mat_transl = rotz(pi)* transl(actualPose(1), actualPose(2), actualPose(3));
        
                        actualPose = actualPose_mat_transl(1:3,4);
                        app.updateDegreesCurrentConfig(actualConfig);
                        app.updateCartesianCurrentConfig(actualConfig);
        
                        if app.toggleAnimation
                            VisualUtils.showRobotEnvPanel( ...
                                app.SimulationPanel, app.robot, ...
                                app.robotCurrentConfig, app.env);
                        end
                    end
        
                    positionError = norm(actualPose(1:3) - pose(1:3).');

                    if positionError < toleranceErrorMeters
                        validPosition = true;
                        pause(0.5);
                    end
        
                end
        
                status = 0;
        
            catch ME
                status = -1;
                uialert(app.UIFigure, ME.message, "Linear movement error");
            end
        
        end

        function stopRTDERobot(robotRTDEClient, robotPort, acc)
            cmd = sprintf('stopl(%f)', acc);
            executeURScriptCommand(robotRTDEClient, cmd, robotPort);
        end

        function executeMovement(app, interpPath, goalVector, status, targetPose)  
            switch app.operationMode
                case "Simulation"
                    if status == 0
                        VisualUtils.animatePath( ...
                            app, app.robotCurrentConfig, interpPath, app.simulationSecs);
                        app.robotCurrentConfig = app.robotDesiredConfig;
                    end
                case "Simulate & Move"
                    VisualUtils.animatePath( ...
                        app, app.robotCurrentConfig, interpPath, app.simulationSecs);
    
                    app.robotCurrentConfig = app.robotDesiredConfig;
    
                    if status == 0
                        try
                            if (strcmp(app.movementMode, "Movej"))
                                MovementUtils.moveRTDERobotJoints( ...
                                    app.robotRTDEClient, app.robotPort, goalVector, ...
                                    app.toleranceErrorDegrees, app.acc, app.speed, app);
                            else
                                MovementUtils.moveRTDERobotLinear(app.robotRTDEClient, app.robotPort, ...
                                    targetPose, app.toleranceErrorMeters, app.acc, app.speed, app);
                            end
                        catch Error
                            uialert(app.UIFigure, Error.message, "Movement error");
                            return;
                        end
                    end
                case "Move"
                    try
                        if (strcmp(app.movementMode, "Movej"))
                            MovementUtils.moveRTDERobotJoints( ...
                                app.robotRTDEClient, app.robotPort, goalVector, ...
                                app.toleranceErrorDegrees, app.acc, app.speed, app);
                        else
                            MovementUtils.moveRTDERobotLinear(app.robotRTDEClient, app.robotPort, ...
                                    targetPose, app.toleranceErrorMeters, app.acc, app.speed, app);
                        end
                    catch
                        uialert(app.UIFigure, ...
                            "ERROR: Can't move robot joints", ...
                            "Movement error");
                        MovementUtils.stopRTDERobot( ...
                            app.robotRTDEClient, app.robotPort, app.acc);
                        return;
                    end
            end
        end
    end 
end