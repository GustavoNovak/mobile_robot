classdef Images
    properties (Access = private)
    end
    
    methods (Static)  
        function topographicMap = generateTopographicMap(cameras, img)
            img = rgb2gray(img);
            BW1 = edge(img,'sobel');
            figure;
            imshow(BW1);
            se90 = strel('line',15,90);
            se0 = strel('line',15,0);
            BWsdil = imdilate(BW1,[se90 se0]);
            BWdfill = imfill(BWsdil,'holes');
            img = bwareafilt(BWdfill, 3);
            imageSize = size(img);
            for i=1:imageSize(1)
                for j=1:imageSize(2)
                    realPosition = cameras.getRealPosition([j i], 0);
                    if((realPosition(1) < -1390 || realPosition(1) > 1390) || (realPosition(2) < -1450 || realPosition(2) > 1450))
                        img(i,j) = 1;
                    end
                end            
            end
            imwrite(imcomplement(img),'storage/real_topographic_map.png')
            
            imshow(img);
            imgToVerify = img;
            
            for i=1:imageSize(1)
                for j=1:imageSize(2)
                    if (imgToVerify(i,j) == 1)
                        if(BWsdil(i,j) == 1)
                            circleCenter = cameras.getRealPosition([j i], 0);
                            for theta = 0:(pi/10):2*pi
                                circlePosition = circleCenter + 185*[cos(theta) sin(theta) 0];
                                circlePosition = [circlePosition(1) ; circlePosition(2) ; circlePosition(3)];
                                realPixelPosition = cameras.getRealPixelPosition(circlePosition);
                                realPixelPosition(1) = floor(realPixelPosition(1));
                                realPixelPosition(2) = floor(realPixelPosition(2));
                                if((realPixelPosition(1) >= 1 && realPixelPosition(1) <= imageSize(2)) && (realPixelPosition(2) >= 1 && realPixelPosition(2) <= imageSize(1)))
                                    img(realPixelPosition(2), realPixelPosition(1)) = 1;
                                end
                            end                            
                        end
                    end
                end     
            end
            
            img = imfill(img,'holes');
            figure;
            seD = strel('diamond',1);
            img = imerode(img,seD);
            img = imerode(img,seD);
            img = imcomplement(img);
            
            tolerance = 175 + 150;

            for i=1:imageSize(1)
                for j=1:imageSize(2)
                    realPosition = cameras.getRealPosition([j i], 0);
                    if((realPosition(1) < -(1390 - tolerance) || realPosition(1) > (1390 - tolerance)) || (realPosition(2) < -(1450 - tolerance) || realPosition(2) > (1450 - tolerance)))
                        img(i,j) = 0;
                    end
                end            
            end
            
            imwrite(img,'storage/topographic_map.png')
            imshow(img);
        end
        
        function [count, xPixel, yPixel, countSettedVelocity] = getCirclePositon(image, color, pixelPosition, cameraNumber, velocity, timer, countSettedVelocity, robot)
            if (strcmp(color, 'blue')) 
                xPixel = 0;
                yPixel = 0;
            else
                xPixel = 150;
                yPixel = 150;
            end
%             figure;
%             imshow(image);
%             impixelinfo;
            imageValues = im2double(image);
            xPixel = 0;
            yPixel = 0;
            imageSize = size(image);
            points = [];
            count = 0;
            if(cameraNumber == 1)
                greenIndicator = 0.15;
                redIndicator = 0.15;     
            else
                greenIndicator = 0.15;
                redIndicator = 0.25;                
            end
            
            for i=(pixelPosition(2)-100):(pixelPosition(2)+100)
                t = toc(timer);
                if((t - countSettedVelocity*0.2) > 0)
                    countSettedVelocity = countSettedVelocity + 1;
                    robot.setVelocity(velocity);
                end
                for j=(pixelPosition(1)-100):(pixelPosition(1)+100)
                    if((i >= 1 && i <= imageSize(1)) && (j >= 1 && j <= imageSize(2)))
                        if (strcmp(color, 'green'))
                            indicator = imageValues(i,j,2) - ((imageValues(i,j,1) + imageValues(i,j,3))/2);
                            if (indicator > greenIndicator) 
                                count = count + 1;
                                points(count, :) = [i j];
    %                             for k1 = i:(i+15)
    %                                 for k2 = j:(j+15)
    %                                     image(k1,k2,1) = 255;
    %                                     image(k1,k2,2) = 255;
    %                                     image(k1,k2,3) = 255; 
    %                                 end
    %                             end
%                                 image(i,j,1) = 255;
%                                 image(i,j,2) = 255;
%                                 image(i,j,3) = 255; 
                            end
                        elseif (strcmp(color, 'red'))
                            if(cameraNumber == 1)
                                indicator = imageValues(i,j,1) - ((imageValues(i,j,2) + imageValues(i,j,3))/2.2);
                                if (indicator > redIndicator)
                                    count = count + 1;
                                    points(count, :) = [i j];
        %                             for k1 = i:(i+15)
        %                                 for k2 = j:(j+15)
        %                                     image(k1,k2,1) = 255;
        %                                     image(k1,k2,2) = 255;
        %                                     image(k1,k2,3) = 255; 
        %                                 end
%         %                             end
%                                     image(i,j,1) = 255;
%                                     image(i,j,2) = 255;
%                                     image(i,j,3) = 255; 
                                end                               
                            else 
                                indicator = imageValues(i,j,1) - ((imageValues(i,j,2) + imageValues(i,j,3))/2.2);
                                if (indicator > redIndicator)
                                    count = count + 1;
                                    points(count, :) = [i j];
        %                             for k1 = i:(i+15)
        %                                 for k2 = j:(j+15)
        %                                     image(k1,k2,1) = 255;
        %                                     image(k1,k2,2) = 255;
        %                                     image(k1,k2,3) = 255; 
        %                                 end
        %                             end
%                                     image(i,j,1) = 255;
%                                     image(i,j,2) = 255;
%                                     image(i,j,3) = 255; 
                                end   
                            end
                        end 
                    end
                end
            end
            
%             if(count == 0)
% %                 disp('não achou de primeira');
%                 for i=1:imageSize(1)
%                     t = toc(timer);
%                     if((t - countSettedVelocity*0.2) > 0)
%                         countSettedVelocity = countSettedVelocity + 1;
%                         robot.setVelocity(velocity);
%                     end
%                     for j=1:imageSize(2)
%                         if (strcmp(color, 'green'))
%                             indicator = imageValues(i,j,2) - ((imageValues(i,j,1) + imageValues(i,j,3))/2);
%                             if (indicator > greenIndicator) 
%                                 count = count + 1;
%                                 points(count, :) = [i j];
%     %                             for k1 = i:(i+15)
%     %                                 for k2 = j:(j+15)
%     %                                     image(k1,k2,1) = 255;
%     %                                     image(k1,k2,2) = 255;
%     %                                     image(k1,k2,3) = 255; 
%     %                                 end
%     %                             end
% %                                 image(i,j,1) = 255;
% %                                 image(i,j,2) = 255;
% %                                 image(i,j,3) = 255; 
%                             end
%                         elseif (strcmp(color, 'red'))
%                             if(cameraNumber == 1)
%                                 indicator = imageValues(i,j,1) - ((imageValues(i,j,2) + imageValues(i,j,3))/2.2);
%                                 if (indicator > redIndicator)
%                                     count = count + 1;
%                                     points(count, :) = [i j];
%         %                             for k1 = i:(i+15)
%         %                                 for k2 = j:(j+15)
%         %                                     image(k1,k2,1) = 255;
%         %                                     image(k1,k2,2) = 255;
%         %                                     image(k1,k2,3) = 255; 
%         %                                 end
%         %                             end
% %                                     image(i,j,1) = 255;
% %                                     image(i,j,2) = 255;
% %                                     image(i,j,3) = 255; 
%                                 end                               
%                             else 
%                                 indicator = imageValues(i,j,1) - ((imageValues(i,j,2) + imageValues(i,j,3))/2.2);
%                                 if (indicator > redIndicator)
%                                     count = count + 1;
%                                     points(count, :) = [i j];
%         %                             for k1 = i:(i+15)
%         %                                 for k2 = j:(j+15)
%         %                                     image(k1,k2,1) = 255;
%         %                                     image(k1,k2,2) = 255;
%         %                                     image(k1,k2,3) = 255; 
%         %                                 end
%         %                             end
% %                                     image(i,j,1) = 255;
% %                                     image(i,j,2) = 255;
% %                                     image(i,j,3) = 255; 
%                                 end   
%                             end
%                         end 
%                     end
%                 end
%             end

            if (count > 1)
                sumX = 0;
                sumY = 0;
                for i = 1:count
                    sumX = sumX + points(i, 2);
                    sumY = sumY + points(i, 1);
                end
                            
                xPixel = sumX/length(points);
                yPixel = sumY/length(points);
            else
                xPixel = -999999;
                yPixel = -999999;
            end
%             for k1 = (xPixel-1):(xPixel+1)
%                 for k2 = (yPixel-1):(yPixel+1)
%                     i = floor(k1);
%                     j = floor(k2);
%                     image(j,i,1) = 0;
%                     image(j,i,2) = 0;
%                     image(j,i,3) = 0; 
%                 end
%             end
%             
%             figure;
%             imshow(image);
%             impixelinfo
        end
        
        function pixelPoints = calculatePointsInImage(calibrationPoints, focusLength, imageDensity, pixelCenter)
            F = [
                focusLength 0 0
                0 focusLength 0
                0 0 1
            ];
            M = [
                imageDensity(1) 0 ;
                0 imageDensity(2) ;
            ];
            
            theta(1) = 0;
            psi(1) = 0.5;
            phi(1) = -0.5;
            t(1,1) = 25;
            t(2,1) = 200;
            t(3,1) = 1000;
            
            count = 0;
            for i = 1:length(theta)
                R = services.Math.getRotationMatrix(psi(i), theta(i), phi(i));
                T = [
                    R(1,1) R(1,2) R(1,3) t(1,i) ; 
                    R(2,1) R(2,2) R(2,3) t(2,i) ; 
                    R(3,1) R(3,2) R(3,3) t(3,i) ; 
                    0 0 0 1
                ]
                for j = 1:length(calibrationPoints)
                    cameraCoordenates = T*[calibrationPoints(j,1) ;
                                           calibrationPoints(j,2) ; 
                                           calibrationPoints(j,3) ; 
                                           1];
                    sensorCoordenates = F*[cameraCoordenates(1) ;
                                           cameraCoordenates(2) ;
                                           cameraCoordenates(3)];
                    pixelValues = (1/sensorCoordenates(3))*M*[sensorCoordenates(1) ; sensorCoordenates(2)] + [pixelCenter(1) ; pixelCenter(2)];
                    count = count + 1;
                    pixelPoints(count,:) = [i pixelValues(1) pixelValues(2)]; 
                end
            end
            
%             function getCalibrationPoints(cameras)
% %                 img = 
%                  
%             end
        end
        
        function [calibrationPoints, pixelValuesCalibrationPoints, imageSize] = getCalibrationPoints(cameraNumber, cameras, chamberSize)
            img = getsnapshot(cameras(cameraNumber));
            imgSize = size(img);
            figure;
            imshow(img);
            impixelinfo;
            img = im2double(img);
            imageSize = size(img);
            imageSize = [imageSize(2) imageSize(1)];
            
            redImage = zeros(imgSize(1), imgSize(2));
            for i = 1:imgSize(1)
                for j = 1:imgSize(2)
                    value = img(i,j,1) - ((img(i,j,2) + img(i,j,3))/2);
                    if(value < 0)
                        value = 0;
                    end
                    redImage(i,j,1) = value;
                    redImage(i,j,2) = 0;
                    redImage(i,j,3) = 0;
                end
            end
            figure;
            imshow(redImage);
            impixelinfo;
            figure;
            edgeImg = edge(rgb2gray(redImage), 'Sobel');
%             se90 = strel('line',3,90);
%             se0 = strel('line',3,0);
%             edgeImg = imdilate(edgeImg,[se90 se0]);
%             edgeImg = imfill(edgeImg,'holes');
            imshow(edgeImg);
            [H,T,R] = hough(edgeImg, 'RhoResolution', 0.5, 'Theta', -90:0.01:89.9);
            figure;
            imshow(H,[],'XData',T,'YData',R,...
                        'InitialMagnification','fit');
            xlabel('\theta'), ylabel('\rho');
            axis on, axis normal, hold on;
            solutionFound = false;
            for i = 4:8
                P  = houghpeaks(H,i,'threshold',ceil(0.3*max(H(:))));
                x = T(P(:,2));
                y = R(P(:,1));
                points = [];
                count = 0;
                for j = 1:i
                    if(count > 0)
                        keep = false;
                        for k = 1:count
                            distance = [x(j) 10*y(j)] - [points(k, 1) 10*points(k, 2)];
                            if(norm(distance) < 300)
                                keep = true;
                                break;
                            end
                        end
                        
                        if(~keep)
                            count = count + 1;
                            points(count, :) = [x(j) y(j)];
                        end
                    else
                        count = count + 1;
                        points(count, :) = [x(j) y(j)];
                    end
                end
                
                if(count == 4)
                    solutionFound = true;
                    break;
                end
            end
            
            countIntersectionPoints = 0;
            intersectionPoints = [];
            if(solutionFound)
                plot(points(:, 1), points(:, 2), 's', 'color', 'white');

                for i = 1:4
                    angleDistances = [];
                    count = 0;
                    for j = 1:4
                        count = count + 1;
                        angleDistances(count, :) = [j abs(points(i, 1) - points(j, 1))];
                    end
                    
                    perpendicularLines = [];
                    count = 0;
                    for j = 1:4
                        if(count == 2)
                            for k = 1:count
                                if(perpendicularLines(k, 2) < angleDistances(j, 2))
                                    perpendicularLines(k, :) = angleDistances(j, :);
                                    break;
                                end
                            end
                        else
                            count = count + 1;
                            perpendicularLines(count, :) = angleDistances(j, :);
                        end
                    end

                    for j = 1:2
                        for k = 1:4
                            if(perpendicularLines(j, 1) == k)
                                perpendicularLines(j, :) = points(k, :);
                            end
                        end
                    end
                    
                    for j = 1:2
                        theta1 = (pi/180)*points(i, 1);
                        theta2 = (pi/180)*perpendicularLines(j, 1);
                        ro1 = points(i, 2);
                        ro2 = perpendicularLines(j, 2);
                        
                        countIntersectionPoints = countIntersectionPoints + 1;
                        intersectionPoints(countIntersectionPoints, :) = (1/sin(theta2 - theta1))*[(ro1*sin(theta2) - ro2*sin(theta1)) (-ro1*cos(theta2) + ro2*cos(theta1))];
                    end
                end
                
                count = 0;
                finalIntersectionPoints = [];
                for j = 1:8
                    if(count > 0)
                        canRegister = true;
                        for k = 1:count
                            if(finalIntersectionPoints(k, 1) == intersectionPoints(j, 1) && finalIntersectionPoints(k, 2) == intersectionPoints(j, 2))
                                canRegister = false;
                                break;
                            end
                        end
                        
                        if(canRegister)
                            count = count + 1;
                            finalIntersectionPoints(count, :) = intersectionPoints(j, :);                                     
                        end
                    else
                        count = count + 1;
                        finalIntersectionPoints(count, :) = intersectionPoints(j, :); 
                    end
                end
            else
                msgbox('One of four lines was not detected in Hough transform');
            end
            
            pixelValuesCalibrationPoints = finalIntersectionPoints;
       
            center = [sum(pixelValuesCalibrationPoints(:, 1)/4) sum(pixelValuesCalibrationPoints(:, 2)/4)];
            
            count = 0;
            calibrationPoints = [];
            for i = 1:4    
                aux = sign(pixelValuesCalibrationPoints(i, :) - center);
                
                if(cameraNumber == 1)
                    if(aux(1) > 0 && aux(2) > 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[chamberSize(1) -chamberSize(2) 0];
                    elseif(aux(1) < 0 && aux(2) > 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[-chamberSize(1) -chamberSize(2) 0];
                    elseif(aux(1) < 0 && aux(2) < 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[-chamberSize(1) chamberSize(2) 0];
                    elseif(aux(1) > 0 && aux(2) < 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[chamberSize(1) chamberSize(2) 0];
                    end               
                elseif(cameraNumber == 2)
                    if(aux(1) > 0 && aux(2) > 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[chamberSize(1) chamberSize(2) 0];
                    elseif(aux(1) < 0 && aux(2) > 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[chamberSize(1) -chamberSize(2) 0];
                    elseif(aux(1) < 0 && aux(2) < 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[-chamberSize(1) -chamberSize(2) 0];
                    elseif(aux(1) > 0 && aux(2) < 0) 
                        count = count + 1;
                        calibrationPoints(count, :) = 0.5*[-chamberSize(1) chamberSize(2) 0];
                    end
                end
            end
            
        end
    end
end

