classdef Index
    properties (Access = private)
    end
    
    methods (Static)
        function Index = Index() 
        end
        
        function topographicMap = generateTopographicMap(cameras)
            camerasClass = classes.Cameras();
            img1 = getsnapshot(cameras(1));
            img2 = getsnapshot(cameras(2));
            topographicMap1 = camerasClass.generateTopographicMap(img1, [2780 2900], 1);
            topographicMap2 = camerasClass.generateTopographicMap(img2, [2780 2900], 2);
            topographicMap = zeros(2900,2780);
            for i = 1:2900
                for j = 1:2780
                    if(topographicMap1(i,j) == 1 || topographicMap2(i,j) == 1)
                        topographicMap(i,j) = 1;
                    end
                end
            end
            imwrite(topographicMap,'storage/real_topographic_map.png');
            
            imageToVerify = topographicMap;
            for i=1:2900
                for j=1:2780
                    if (imageToVerify(i,j) == 0)
                        for theta = 0:(pi/20):2*pi
                            circlePosition = [i j] + 185*[cos(theta) sin(theta)];
                            circlePosition(1) = floor(circlePosition(1));
                            circlePosition(2) = floor(circlePosition(2));
                            if((circlePosition(1) >= 1 && circlePosition(1) <= 2900) && (circlePosition(2) >= 1 && circlePosition(2) <= 2780))
                                topographicMap(circlePosition(1), circlePosition(2)) = 0;
                            end
                        end                            
                    end
                end     
            end
            
            imwrite(topographicMap,'storage/topographic_map.png');
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
            measurement = classes.Measurement('mesh', [800 880], [2400 2800]);
            topographicMap = classes.TopographicMap(robot);
            imagesc([-1390 1390], [-1450 1450], topographicMap.map);
            hold on;
            path = classes.Path(topographicMap, measurement, robot);
            [x, y, phi] = robot.getPosition([0 0], camerasClass, cameras);
            points = path.generate([x y phi], camerasClass);
            services.Storage.storePath(points);
        end
        
        function startMeasurement(cameras) 
            preview(cameras(1));
            preview(cameras(2));
            
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
            imagesc([-1390 1390], [-1450 1450], topographicMap.realMap);
            sizePath = size(path);
            for i = 1:(sizePath(1) - 1)
                plot([path(i, 1) path(i+1, 1)], [path(i, 2) path(i+1, 2)], '*-');
            end
            
            robotController = classes.RobotController(robot, path);
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
