classdef TopographicMap
    properties
        map;
        realMap;
        robot;
    end
    
    methods
        function T = TopographicMap(robot)
            T.map = services.Storage.getTopographicMap;
            T.realMap = services.Storage.getRealTopographicMap;
            T.robot = robot;
        end
        
        function response = isFree(T, robotPosition)
            pixelPosition = T.robot.cameras.getRealPixelPosition([robotPosition(1) ; robotPosition(2) ; 0]);
            pixelPosition = [floor(pixelPosition(1)) floor(pixelPosition(2))]; 
            response = false;
            mapSize = size(T.map);
            if ((pixelPosition(1) > 1 && pixelPosition(2) > 1) && (pixelPosition(2) <= mapSize(1) && pixelPosition(1) <= mapSize(2)))
                if (T.map(pixelPosition(2), pixelPosition(1)) == 1)
                    response = true;
                end
            end

            if (response && ~isempty(T.robot.toolZone))
                lengthToolZone = size(T.robot.toolZone);
                lengthToolZone = lengthToolZone(1);
                if(lengthToolZone > 0)
                    for i = 1:lengthToolZone
                        point = [robotPosition(1) robotPosition(2)] + [cos(robotPosition(3) + T.robot.toolZone(i,1))*T.robot.toolZone(i,2) sin(robotPosition(3) + T.robot.toolZone(i,1))*T.robot.toolZone(i,2)];
                        pixelPosition = T.robot.cameras.getRealPixelPosition([point(1) ; point(2) ; 0]);
                        pixelPosition = [floor(pixelPosition(1)) floor(pixelPosition(2))];  
                        if ((pixelPosition(1) > 1 && pixelPosition(2) > 1) && (pixelPosition(2) <= mapSize(1) && pixelPosition(1) <= mapSize(2)))
                            if (T.realMap(pixelPosition(2), pixelPosition(1)) == 0)
                                response = false;
                                break;
                            end
                        else
                            response = false;
                        end  
                    end
                end
            end
        end
        
        function display(T)
            disp('topographicMap: '); disp(T.map);
            disp('realTopographicMap: '); disp(T.realMap);
            disp('robot: '); disp(T.robot);
        end
        
        % Setters
        function T = set.map(T, value)
            if (services.Validator.isImage(value, ''))
                T.map = value;
            else
                error('Invalid image');
            end
        end
        function T = set.realMap(T, value)
            if (services.Validator.isImage(value, ''))
                T.realMap = value;
            else
                error('Invalid image');
            end
        end
        function T = set.robot(T, value)
            if (services.Validator.isClass(value, 'classes.Robot'))
                T.robot = value;
            else
                error('Invalid robot');
            end
        end
    end
end
