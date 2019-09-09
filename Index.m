classdef Index
    properties (Access = private)
    end
    
    methods (Static)
        function Index = Index() 
        end
        
        function generateTopographicMap()
            clear all;
            close all;
            try
                cameras = classes.Cameras();
                img = getsnapshot(cameras.cameras(2));
                services.Images.generateTopographicMap(cameras, img);
                cameras.delete();
            catch e
                cameras.delete();
                fprintf(1,'There was an error! The message was:\n%s',e.message);                
            end
        end
        
        function testPID(position)
            system('start /min interface_cpp.exe');
            robot = classes.Robot(300, [430 40 0], [430 0], 60, 0.1); 
            try
                topographicMap = classes.TopographicMap(robot);
                preview(robot.cameras.cameras(2));
                imagesc([-1000 1000], [-1350 1350], topographicMap.map);
                hold on;
                robot.getPosition();
                plot(-175, 0, 'o');
                robotController = classes.RobotController(robot, position);
                robotController.moveInPath()
                robot.cameras.delete();
                system('TASKKILL -f -im "interface_cpp.exe"');
            catch e
                robot.cameras.delete();
                system('TASKKILL -f -im "interface_cpp.exe"');
                fprintf(1,'There was an error! The message was:\n%s',e.message);                
            end
        end
        
        function moveRobot(V)
            system('start /min interface_cpp.exe');
            robot = classes.Robot(300, [10 0], 60, 0.1);
            robot.setVelocity(V, 0);
            robot.cameras.delete();
            system('TASKKILL -f -im "interface_cpp.exe"');
        end
        
        function generatePath(cameras)
            robot = classes.Robot(300, [430 40 0], [430 0], 90, 0.15);    
            camerasClass = classes.Cameras();
            measurement = classes.Measurement('mesh', [800 880], [-1200 1400 ; 1200 1400 ; 1200 -1400 ; -1200 -1400]);
            topographicMap = classes.TopographicMap(robot);
            imagesc([0 640], [0 480], topographicMap.map);
            hold on;
            path = classes.Path(topographicMap, measurement, robot);
            [x, y, phi] = robot.getPosition([1 1], camerasClass);
            points = path.generate([x y phi], camerasClass);
            services.Storage.storePath(points);
        end
        
        function startMeasurement(cameras) 
            system('start /min interface_cpp.exe');
            robot = classes.Robot(300, [430 40 0], [430 0], 90, 0.15);    
            try
                path = services.Storage.getPath();
            catch e
                close all;
                msgbox('Before start measurement you need to generate a path');
                system('TASKKILL -f -im "interface_cpp.exe"');
                return;
            end
            
            camerasClass = classes.Cameras();
            topographicMap = classes.TopographicMap(robot);
            imagesc([0 640], [0 480], topographicMap.map);
            sizePath = size(path);
            for i = 1:(sizePath(1) - 1)
                startPixelPosition = camerasClass.getRealPixelPosition([path(i, 1) ; path(i, 2) ; 0]);
                endPixelPosition = camerasClass.getRealPixelPosition([path(i+1, 1) ; path(i+1, 2) ; 0]);
                plot([startPixelPosition(1) endPixelPosition(1)], [startPixelPosition(2) endPixelPosition(2)], '*-');
            end
            
            for i = 1:lengthPoints
                pixelPosition = camerasClass.getRealPixelPosition([points(i, 1) ; points(i, 2) ; 0]);
                plot(pixelPosition(1), pixelPosition(2), 'o');
            end
            
            robotController = classes.RobotController(robot, points);
            robotController.moveInPath(camerasClass, cameras)
            disp('Measurements finished!');
            system('TASKKILL -f -im "interface_cpp.exe"');
        end
        
        function showTopographicMap() 
            map = imread('storage/topographic_map.tif');
            figure;imshow(map);
            impixelinfo
        end
        
        function display(R)

        end
    end
end
