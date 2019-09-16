classdef TopographicMap
    properties
        map;
        realMap;
        robot;
    end
    
    methods
        function T = TopographicMap(robot)
            T.map = services.Storage.getTopographicMap();
            T.realMap = services.Storage.getRealTopographicMap();
            T.robot = robot;
        end
        
        function response = isFree(T, robotPosition)
            robotPosition = [floor(robotPosition(1)) floor(robotPosition(2)) robotPosition(3)]; 
            response = false;
            mapSize = size(T.map);
            if ((robotPosition(1) > -mapSize(2)/2 && robotPosition(2) > -mapSize(1)/2) && (robotPosition(2) < mapSize(1)/2 && robotPosition(1) < mapSize(2)/2))
                if (T.map(robotPosition(2) + mapSize(1)/2, robotPosition(1) + mapSize(2)/2) == 1)
                    response = true;
                end
            end

            if (response && ~isempty(T.robot.toolZone))
                lengthToolZone = size(T.robot.toolZone);
                lengthToolZone = lengthToolZone(1);
                if(lengthToolZone > 0)
                    for i = 1:lengthToolZone
                        point = [robotPosition(1) robotPosition(2)] + [cos(robotPosition(3) + T.robot.toolZone(i,1))*T.robot.toolZone(i,2) sin(robotPosition(3) + T.robot.toolZone(i,1))*T.robot.toolZone(i,2)];
                        point = [floor(point(1)) floor(point(2))];  
                        if ((point(1) > -mapSize(2)/2 && point(2) > -mapSize(1)/2) && (point(2) < mapSize(1)/2 && point(1) < mapSize(2)/2))
                            if (T.realMap(point(2)  + mapSize(1)/2, point(1)  + mapSize(2)/2) == 0)
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
