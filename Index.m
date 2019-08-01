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
        
        function startMeasurement() 
            clear all;
            close all;
            system('start /min interface_cpp.exe');
            toolZone = [
                0.1538 195.860 ; 
                0.1538 183.743 ; 
                0.1149 203.147 ; 
                0.1149 231.917 ; 
                0.1149 261.725 ; 
                0.0564 196.792 ; 
                0.0564 223.346 ; 
                0.0564 243.951 ; 
                0.0564 260.414 ; 
                0.0000 260.000 ; 
                0.0000 243.562 ; 
                0.0000 222.990 ;  
                0.0000 196.479 ; 
                -0.1538 195.860 ; 
                -0.1538 183.743 ; 
                -0.1149 203.147 ; 
                -0.1149 231.917 ; 
                -0.1149 261.725 ; 
                -0.0564 196.792 ; 
                -0.0564 223.346 ; 
                -0.0564 243.951 ; 
                -0.0564 260.414 ; 
                ];
            robot = classes.Robot(300, [430 40 0], [430 0], 60, 0.1);    
            try
                measurement = classes.Measurement('mesh', [300 250], [-1000 1000 ; 1000 1000 ; 1000 -1000 ; -1000 -1000]);
                topographicMap = classes.TopographicMap(robot);
                imagesc([0 640], [0 480], topographicMap.map);
                hold on;
                path = classes.Path(topographicMap, measurement, robot);
                points = path.generate();
                disp('length of points: ');
                disp(length(points))
                hold on;
                robotController = classes.RobotController(robot, points);
                robotController.moveInPath()
                robot.cameras.delete();
                disp('Measurements finished!');
                system('TASKKILL -f -im "interface_cpp.exe"');
            catch e
                robot.cameras.delete();
                system('TASKKILL -f -im "interface_cpp.exe"');
                fprintf(1,'There was an error! The message was:\n%s',e.message);
            end
%             for k = 1:(length(points) - 1)
%                 disp(k);
%                 distance = (points(k+1,:) - points(k,:))/1000;
%                 normDistance = norm(distance);
%                 velocity = 0.1*(distance/normDistance);
%                 while(normDistance > 0)
%                     robot.setVelocity([velocity(1) velocity(2) 0]);
%                     normDistance = normDistance - 0.02;
%                     pause(0.2);
%                 end
%             end
        end
        
        function generatePath()
            robot = classes.Robot(300, [100 100], 100, 0.2);
            measurement = classes.Measurement('mesh', [25 25], [-100 100 ; 100 100 ; 100 -100 ; -100 -100]);
            map = imread('storage/topographic_map.tif');
            topographicMap = classes.TopographicMap(map, robot);
            path = classes.Path(topographicMap, measurement);
            
            points = path.generate()
            disp('length of points: ');
            disp(length(points))
            for k = 1:(length(points) - 1)
                disp(k);
                distance = (points(k+1,:) - points(k,:))/1000;
                normDistance = norm(distance);
                velocity = 0.1*(distance/normDistance);
                while(normDistance > 0)
                    robot.setVelocity([velocity(1) velocity(2) 0]);
                    normDistance = normDistance - 0.02;
                    pause(0.2);
                end
            end
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
