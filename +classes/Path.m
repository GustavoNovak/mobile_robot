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
        
        function path = generate(P)
            % Transform measurement point in robot positions and delete not reachable points
            measurementPoints = P.measurement.getMeasurementPoints();
            disp('Measurement Points: ');
            disp(measurementPoints);
            
            count = 0;
            i = 1;
            while i <= length(measurementPoints)
                theta = -pi;
                while theta <= pi  
                    robotPosition = services.Math.getRobotPosition(measurementPoints(i, :), theta, P.topographicMap.robot);
          
                    if (P.topographicMap.isFree(robotPosition))
                        count = count + 1;
                        points(count, :) = robotPosition;
                        break;
                    end 
                
                    theta = theta + (2*pi/36);
                end 
                
                i = i + 1;
            end
            disp('Robot Positions in Measurement Points: ');
            disp(points);
               
            [x, y, phi] = P.robot.getPosition()
            
            path1 = services.PathPlanning.generatePath([x y phi], points(1, :), P.topographicMap, P.measurement);
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
                    newPath = services.PathPlanning.generatePath(points(j, :), points(j+1, :), P.topographicMap, P.measurement);
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
            disp('Generated Path');
            path
            for i = 1:(length(path) - 1)
                startPixelPosition = P.robot.cameras.getRealPixelPosition([path(i, 1) ; path(i, 2) ; 0]);
                endPixelPosition = P.robot.cameras.getRealPixelPosition([path(i+1, 1) ; path(i+1, 2) ; 0]);
                plot([startPixelPosition(1) endPixelPosition(1)], [startPixelPosition(2) endPixelPosition(2)], '*-');
            end
            
            for i = 1:lengthPoints
                pixelPosition = P.robot.cameras.getRealPixelPosition([points(i, 1) ; points(i, 2) ; 0]);
                plot(pixelPosition(1), pixelPosition(2), 'o');
            end
        end
        
        function display(P)
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
