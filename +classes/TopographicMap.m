classdef TopographicMap
    properties
        map;
        robot;
    end
    
    methods
        function T = TopographicMap(robot)
            T.map = services.Storage.getTopographicMap;
            T.robot = robot;
        end
        
        function response = isFree(T, point)
            pixelPosition = T.robot.cameras.getRealPixelPosition([point(1) ; point(2) ; 0])
            pixelPosition = [floor(pixelPosition(1)) floor(pixelPosition(2))]; 
            response = false;
            mapSize = size(T.map);
            if ((pixelPosition(1) > 1 && pixelPosition(2) > 1) && (pixelPosition(2) <= mapSize(1) && pixelPosition(1) <= mapSize(2)))
                if (T.map(pixelPosition(2), pixelPosition(1)) == 1)
                    response = true;
                end
            end
        end
        
        function display(T)
            disp('image: '); disp(T.map);
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
        function T = set.robot(T, value)
            if (services.Validator.isClass(value, 'classes.Robot'))
                T.robot = value;
            else
                error('Invalid robot');
            end
        end
    end
end
