classdef Path
    properties
        topographicMap;
        measurement;
        robot;
    end
    
    methods
        function P = Path(topographicMap, measurement, robot)
            P.topographicMap = topographicMap;
            P.measurement = measurement;
            P.robot = robot;
        end
        
        function path = generate(P, initialPosition, camerasClass)  
            % Transform measurement point in robot positions and delete not reachable points
            measurementPoints = P.measurement.getMeasurementPoints();
            disp('Measurement Points: ');
            disp(measurementPoints);  
            points = [];
            count = 0;
            i = 1;
            while i <= length(measurementPoints)
                theta = 0;
                while theta <= pi
                    robotPosition = services.Math.getRobotPosition(measurementPoints(i, :), -theta, P.robot);
          
                    if (P.topographicMap.isFree(robotPosition))
                        count = count + 1;
                        points(count, :) = robotPosition;
                        break;
                    end 
                    
                    robotPosition = services.Math.getRobotPosition(measurementPoints(i, :), theta, P.robot);
          
                    if (P.topographicMap.isFree(robotPosition))
                        count = count + 1;
                        points(count, :) = robotPosition;
                        break;
                    end 
                
                    theta = theta + pi/100;
                end 
                
                i = i + 1;
            end
            disp('Robot Positions in Measurement Points: ');
            disp(points);
            path1 = services.PathPlanning.generatePath(initialPosition, points(1, :), P.topographicMap, P.measurement, camerasClass);
            count = 0;
            lengthPath1 = size(path1);
            lengthPath1 = lengthPath1(1);
            for i = 1: lengthPath1
                if (i > 1)
                    count = count + 1;
                    if (i < lengthPath1)
                        path(count, :) = [path1(i, 1) path1(i, 2) path1(i, 3) 0];
                    else
                        path(count, :) = [path1(i, 1) path1(i, 2) path1(i, 3) 1];
                    end
                end
            end
            
            lengthPoints = size(points);
            lengthPoints = lengthPoints(1);
            if (lengthPoints > 1)
                for j = 1: (lengthPoints-1)
                    newPath = services.PathPlanning.generatePath(points(j, :), points(j+1, :), P.topographicMap, P.measurement, camerasClass);
                    if(newPath ~= 0)
                        lengthNewPath = size(newPath);
                        lengthNewPath = lengthNewPath(1);                    
                        for i = 1: lengthNewPath
                            if (i > 1)
                                count = count + 1;
                                if (i < lengthNewPath)
                                    path(count, :) = [newPath(i, 1) newPath(i, 2) newPath(i, 3) 0]; 
                                else
                                    path(count, :) = [newPath(i, 1) newPath(i, 2) newPath(i, 3) 1]; 
                                end
                            end
                        end
                    end
                end
            end
            disp('Generated Path');
            sizePath = size(path);
            for i = 1:(sizePath(1) - 1)
                plot([path(i, 1) path(i+1, 1)], [path(i, 2) path(i+1, 2)], '*-');
            end
            
            for i = 1:lengthPoints
                plot(points(i, 1), points(i, 2), 'o');
            end
            
            for i = 1:length(measurementPoints)
                plot(measurementPoints(i, 1), measurementPoints(i, 2), 's');
            end
        end
        
        function P = set.topographicMap(P, value)
            if (services.Validator.isClass(value, 'classes.TopographicMap')) 
                if (~isempty(value.map))
                    P.topographicMap = value;
                else
                    error('TopographicMap passed without map');
                end
            else
                error('Must be TopographicMap class');
            end
        end
        function P = set.measurement(P, value)
            if (services.Validator.isClass(value, 'classes.Measurement')) 
                P.measurement = value;
            else
                error('Must be TopographicMap class');
            end
        end
    end
end
