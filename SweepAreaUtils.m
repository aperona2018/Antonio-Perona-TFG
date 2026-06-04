classdef SweepAreaUtils

    methods (Static)

        function [Xrot, Yrot, Zrot] = defineRotatedPlane(Ny, Nz, yRange, zRange, x, rotAngles)
            rotX = deg2rad(rotAngles(1));
            rotY = deg2rad(rotAngles(2));
            rotZ = deg2rad(rotAngles(3));
        
            y = linspace(yRange(1), yRange(2), Ny);
            z = linspace(zRange(1), zRange(2), Nz);
            [Y1, Z1] = meshgrid(y, z);
        
            Z1(:,1:2:end) = flipud(Z1(:,1:2:end));
        
            Y = Y1(:);
            Z = Z1(:);
            X = ones(size(Y)) * x;
        
            Rx = [1 0 0; 0 cos(rotX) -sin(rotX); 0 sin(rotX) cos(rotX)];
            Ry = [cos(rotY) 0 sin(rotY); 0 1 0; -sin(rotY) 0 cos(rotY)];
            Rz = [cos(rotZ) -sin(rotZ) 0; sin(rotZ) cos(rotZ) 0; 0 0 1];
        
            R = Rz * Ry * Rx;
        
            originalPoints = [X, Y, Z];
            rotatedPoints = (R * originalPoints')';
        
            Xrot = rotatedPoints(:,1);
            Yrot = rotatedPoints(:,2);
            Zrot = rotatedPoints(:,3);
        end

        function correctedTargetPose = correctTargetPose(deviationAnglesRad, targetPose)
            eulerMatrix = inv(eul2rotm(deviationAnglesRad, 'XYZ'));
            correctionMatrix = zeros(4,4);
            correctionMatrix(4,4) = 1;
            correctionMatrix(1:3, 1:3) = eulerMatrix;
        
            correctedTargetPose = targetPose * correctionMatrix;   
        end


        function [deviationAnglesRad] = getOptitrackAngles(optitrackNatnetclient)
            data = optitrackNatnetclient.getFrame;
        
            q = [data.RigidBodies(1).qw, ...
                 data.RigidBodies(1).qx, ...
                 data.RigidBodies(1).qy, ...
                 data.RigidBodies(1).qz];
        
            deviationAnglesRad = quat2eul(q, 'XYZ');             
        end

        function [centerConfig, targetCenterPose] = moveRobotToCenter(app, robot, Xrot, Yrot, Zrot, ...
                planner, weights, startConfig, toolName, UIFigure)
            xCenter = mean(Xrot);
            yCenter = mean(Yrot);
            zCenter = mean(Zrot);
        
            targetCenterPose = MovementUtils.defineIkPose(app, pi/2, xCenter, yCenter, zCenter);
        
            if (strcmp(app.movementMode, "Movej"))
                [goalConfig, solInfo] = MovementUtils.resolveInverseKinematics(robot, targetCenterPose, weights, startConfig, UIFigure, toolName);
        
                startVector = [startConfig.JointPosition];
                goalVector = [goalConfig.JointPosition];

                [interpPath, status] = MovementUtils.resolveMovementDirectDegrees(planner, ...
                    startVector, goalVector, UIFigure);
            else
                interpPath = [];
                status = 0;
                goalVector = [];
                goalConfig = [];
            end
            
            MovementUtils.executeMovement(app, interpPath, goalVector, status, targetCenterPose);
            
            centerConfig = goalConfig;
        end

        function [measurements, Yrot, Zrot] = sweepArea( ...
            app, sweepAreaInfo)

            optitrackNatnetclient = sweepAreaInfo.optitrackNatnetclient;
            robot = sweepAreaInfo.robot;
            env = sweepAreaInfo.env;
            Ny = sweepAreaInfo.Ny;
            Nz = sweepAreaInfo.Nz;
            yRange = sweepAreaInfo.yRange;
            zRange = sweepAreaInfo.zRange;
            x = sweepAreaInfo.x;
            weights = sweepAreaInfo.weights;
            startConfig = sweepAreaInfo.startConfig;
            rotAngles = sweepAreaInfo.rotAngles;
            toolName = sweepAreaInfo.toolName;
            devPNA = sweepAreaInfo.devPNA;
            numPoints = sweepAreaInfo.numPoints;
        
            validationDist = 0.1;
        
            [Xrot, Yrot, Zrot] = SweepAreaUtils.defineRotatedPlane( ...
                Ny, Nz, yRange, zRange, x, rotAngles);
        
            planner = MovementUtils.initializePlanner(robot, env, validationDist);
        
            app.FinishTimeResultLabel.Text = 'N/A';
            app.EstimatedTimeResultLabel.Text = 'N/A';

            [currentConfig, targetCenterPose] = SweepAreaUtils.moveRobotToCenter( ...
                app, robot, Xrot, Yrot, Zrot, ...
                planner, weights, startConfig, ...
                toolName, app.UIFigure);

            deviationAnglesRad = [];
            if ~isempty(optitrackNatnetclient)
                deviationAnglesRad = SweepAreaUtils.getOptitrackAngles( ...
                    optitrackNatnetclient);
            end

            if ~isempty(deviationAnglesRad)
                correctedTargetPose = SweepAreaUtils.correctTargetPose(deviationAnglesRad, targetCenterPose);
                if (strcmp(app.movementMode, "Movej"))
                    [goalConfig, solInfo] = MovementUtils.resolveInverseKinematics(robot, correctedTargetPose, weights, startConfig, app.UIFigure, toolName);
            
                    startVector = [startConfig.JointPosition];
                    goalVector = [goalConfig.JointPosition];
    
                    [interpPath, status] = MovementUtils.resolveMovementDirectDegrees(planner, ...
                        startVector, goalVector, app.UIFigure);
                else
                    goalVector = [];
                    interpPath = [];
                    status = 0;
                end
                MovementUtils.executeMovement(app, interpPath, goalVector, status, correctedTargetPose);
            end
            
            if (~isempty(devPNA))
                measurements = complex(zeros(numel(Xrot), numPoints));
            else
                measurements = 0;
            end
                 targetPose0 = MovementUtils.defineIkPose( ...
                     app, pi/2, mean(Xrot(:)), mean(Yrot(:)), mean(Zrot(:)));

                if ~isempty(deviationAnglesRad)
                    targetPose0 = SweepAreaUtils.correctTargetPose( ...
                        deviationAnglesRad, targetPose0);
                end

            backupFolder = "measurements_backup";

            if ~exist(backupFolder, 'dir')
                mkdir(backupFolder);
            end

            for i = 1:numel(Xrot)
                Xrel=Yrot(i)-mean(Yrot(:));
                Yrel=Zrot(i)-mean(Zrot(:));
                Zrel=Xrot(i)-mean(Xrot(:));
                movimiento_rel=transl(Xrel,Yrel,Zrel);

                targetPose=targetPose0*movimiento_rel;
                
                if (strcmp(app.polarizationMode,"CX"))
                    targetPose = targetPose * rotz(pi/2);
                end

                if (strcmp(app.movementMode, "Movej"))
                    [goalConfig, status] = MovementUtils.resolveInverseKinematics( ...
                        robot, targetPose, weights, currentConfig, app.UIFigure, toolName);
            
                    if status < 0
                        error("IK failed at sweep point %d", i);
                    end
            
                    startVector = [currentConfig.JointPosition];
                    goalVector  = [goalConfig.JointPosition];
            
                    [interpPath, status] = ...
                        MovementUtils.resolveMovementDirectDegrees( ...
                            planner, startVector, goalVector, app.UIFigure);
            
                    if status < 0
                        error("Planning failed at sweep point %d", i);
                    end
            
                    app.robotCurrentConfig = currentConfig;
                    app.robotDesiredConfig = goalConfig;
                else
                    interpPath = [];
                    goalVector = [];
                    status = 0;
                    goalConfig = [];
                end


                MovementUtils.executeMovement(app, interpPath, goalVector, status, targetPose)
                pause(0.2);
                if (i == 1)
                    tic;
                end

                if (i == Nz)
                    app.EstimatedTimeResultLabel.Text = num2str(toc * Ny);
                end

                if ~isempty(devPNA)
                    maxRetries = 3;
                    for attempt = 1:maxRetries
                        acquisitionA = PNAUtils.pnaAcquisition(devPNA);
                        if ~isempty(acquisitionA)
                            measurements(i, :) = acquisitionA;
                            break;
                        end
                    end
                end
                
                if (measurements ~= 0)
                    if mod(i, 2*Nz) == 0 || i == numel(Xrot)
                        measurements_partial = measurements(1:i, :);
                        Yrot_partial = Yrot(1:i);
                        Zrot_partial = Zrot(1:i);
                    
                        save(fullfile(backupFolder, "backup.mat"), ...
                            "measurements_partial", "Yrot_partial", "Zrot_partial");
                    end
                end
                
                currentConfig = goalConfig;
            end
            app.FinishTimeResultLabel.Text = num2str(toc);
        end
    end
end