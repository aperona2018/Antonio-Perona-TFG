classdef Utils
    
    methods(Static)

        function [AppRobotInstance, status, exitMsg] = loadRobot(robotFile)
        
            try
                AppRobotInstance = importrobot(robotFile);
            
                toolBody = getBody(AppRobotInstance, 'tool0');
                safetyBox = collisionBox(0.2, 0.2, 0.2);
                tform = trvec2tform([0, 0, 0.1]); 
                addCollision(toolBody, safetyBox, tform);

                status = 0;
                exitMsg = "Robot imported successfully";
            catch ME
                AppRobotInstance = [];
                status = -1;
                exitMsg = ME.message;
            end
        end

        function [data, status] = readFile(fileRoute)  
            data = [];
            try
                if (isfile(fileRoute))
                    data = load(fileRoute);
                    status = 0;
                else
                    status = -1;
                end
            catch ME
                data = [];
                status = -1;
                return;
            end
        end

        function [robotRTDEClient, status] = connectRTDERobot(robotIP)
            robotRTDEClient = [];
            try
                robotRTDEClient = urRTDEClient(robotIP);
                if isempty(robotRTDEClient)
                    status = -1;
                else
                    status = 0;
                end
            catch ME
                status = -1;
                return;
            end
        end

        function [validConfig] = makeValidConfig(AppRobotInstance, config)
            validConfig = homeConfiguration(AppRobotInstance);
            validConfig(1).JointPosition = config(1).JointPosition;
            validConfig(2).JointPosition = config(2).JointPosition; 
            validConfig(3).JointPosition = config(3).JointPosition; 
            validConfig(4).JointPosition = config(4).JointPosition;
            validConfig(5).JointPosition = config(5).JointPosition; 
            validConfig(6).JointPosition = config(6).JointPosition; 
        end

    end
end