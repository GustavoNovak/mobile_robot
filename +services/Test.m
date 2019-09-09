classdef Test
    properties (Access = private)
    end
    
    methods (Static)
        function generatePath()
            robot = classes.Robot(300, [400 40 0], [400 0], 90, 0.15);    
            camerasClass = classes.Cameras();
            measurement = classes.Measurement('mesh', [750 900], [2400 2800]);
            topographicMap = classes.TopographicMap(robot);
            imagesc([0 640], [0 480], topographicMap.map);
            hold on;
            path = classes.Path(topographicMap, measurement, robot);
            points = path.generate([0 0 0], camerasClass);
            services.Storage.storePath(points);
        end
    end
end