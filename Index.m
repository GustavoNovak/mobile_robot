classdef Index
    properties (Access = private)
    end
    
    methods (Static)
        function Index = Index() 
        end
        
        function [mergedRealTopographicMap, mergedTopographicMap] = generateTopographicMap(cameras, chamberSize, objectsNumber)
            tic;
            camerasClass = classes.Cameras();
%             camerasLength = length(cameras);
            camerasLength = 1;
            
            realTopographicMap = cell(camerasLength, 1);
            topographicMap = cell(camerasLength, 1);
            for i = 1:camerasLength
                if(i == 1)
                    img = getsnapshot(cameras(1));
                else
                    img = getsnapshot(cameras(2));
                end
                [realTopographicMap{i}, topographicMap{i}] = camerasClass.generateTopographicMap(img, chamberSize, objectsNumber, i);
            end
            
            mergedRealTopographicMap = zeros(chamberSize(2), chamberSize(1));
            mergedTopographicMap = zeros(chamberSize(2), chamberSize(1));
            for i = 1:chamberSize(2)
                for j = 1:chamberSize(1)
                    realOccupiedRegion = true;
                    occupiedRegion = true;
                    for k = 1:camerasLength
                        aux1 = realTopographicMap{k};
                        aux2 = topographicMap{k};
                        if(aux1(i,j) == 1)
                            realOccupiedRegion = false;
                        end
                        if(aux2(i,j) == 1)
                            occupiedRegion = false;
                        end
                    end
                    if(realOccupiedRegion)
                        mergedRealTopographicMap(i,j) = 1;
                    end
                    if(occupiedRegion)
                        mergedTopographicMap(i,j) = 1;
                    end
                end
            end
            
            imwrite(imcomplement(mergedRealTopographicMap),'storage/real_topographic_map.png');
            imwrite(imcomplement(mergedTopographicMap),'storage/topographic_map.png');

            figure;
            imshow(imcomplement(mergedTopographicMap));
            impixelinfo;
            figure;
            imshow(imcomplement(mergedRealTopographicMap));
            impixelinfo;
            
            t = toc
        end
        
        function testPID(position, camerasClass, cameras)
            system('start /min interface_cpp.exe');
            robot = classes.Robot(300, [450 40 0], [450 0], 60, 0.1); 
%             try
                topographicMap = classes.TopographicMap(robot);
                preview(cameras(1));
                preview(cameras(2));
                imagesc([-1390 1390], [-1490 1490], topographicMap.map);
                hold on;
                plot(-175, 0, 'o');
                robotController = classes.RobotController(robot, position);
                robotController.moveInPath(camerasClass, cameras);
                system('TASKKILL -f -im "interface_cpp.exe"');
%             catch e
%                 system('TASKKILL -f -im "interface_cpp.exe"');
%                 fprintf(1,'There was an error! The message was:\n%s',e.message);                
%             end
        end
        
        function moveRobot(V)
            system('start /min interface_cpp.exe');
            robot = classes.Robot(300, [10 0], 60, 0.1);
            robot.setVelocity(V, 0);
            robot.cameras.delete();
            system('TASKKILL -f -im "interface_cpp.exe"');
        end
        
%         
%               
% 
        function generatePath(cameras, dx, dy)
            system('start /min interface_cpp.exe');
            figure;
            hold on;
            robot = classes.Robot(300, [450 40 0], [450 0], 90, 0.15);    
%             try
                camerasClass = classes.Cameras();
                measurement = classes.Measurement('mesh', [dx dy], [2780 2800]);
                topographicMap = classes.TopographicMap(robot);
                imagesc([-1390 1390], [-1450 1450], topographicMap.map);
                hold on;
                path = classes.Path(topographicMap, measurement, robot);
                timer = tic;
                [x, y, phi] = robot.getPosition([0 0], camerasClass, cameras, [0 0 0], timer, 0)
                points = path.generate([x y phi], camerasClass);
                services.Storage.storePath(points); 
%             catch e 
%                 close all;
%                 msgbox('One error was found');
                system('TASKKILL -f -im "interface_cpp.exe"');
%                 return;
%             end
        end
        
        function startMeasurement(cameras) 
            preview(cameras(1));
            preview(cameras(2));
            
            system('start /min interface_cpp.exe');
            robot = classes.Robot(300, [450 40 0], [450 0], 90, 0.15);    
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
            figure;
            imagesc([-1390 1390], [-1450 1450], topographicMap.realMap);
            hold on;
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
