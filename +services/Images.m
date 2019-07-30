classdef Images
    properties (Access = private)
    end
    
    methods (Static)  
        function topographicMap = generateTopographicMap(cameras, img)
            img = rgb2gray(img);
            BW1 = edge(img,'sobel');
            figure;
            imshow(BW1);
            se90 = strel('line',5,90);
            se0 = strel('line',5,0);
            BWsdil = imdilate(BW1,[se90 se0]);
            BWdfill = imfill(BWsdil,'holes');
            img = bwareafilt(BWdfill, 3);
            imageSize = size(img);
            
%             count = 0;
%             points = [];
%             for i=1:imageSize(1)
%                 for j=1:imageSize(2)
%                     if (img(i,j) == 1)
%                         circleCenter = cameras.getRealPosition([j i], 0);
%                         for theta = 0:(pi/15):2*pi
%                             circlePosition = circleCenter + 185*[cos(theta) sin(theta) 0];
%                             circlePosition = [circlePosition(1) ; circlePosition(2) ; circlePosition(3)];
%                             realPixelPosition = cameras.getRealPixelPosition(circlePosition);
%                             realPixelPosition(1) = floor(realPixelPosition(1));
%                             realPixelPosition(2) = floor(realPixelPosition(2));
%                             if((realPixelPosition(1) >= 1 && realPixelPosition(1) <= imageSize(2)) && (realPixelPosition(2) >= 1 && realPixelPosition(2) <= imageSize(1)))
%                                 count = count + 1;
%                                 points(count, :) = [realPixelPosition(2) realPixelPosition(1)];
%                             end
%                         end
%                     end
%                 end     
%             end
%             
%             if(count > 0)
%                 for i = 1:length(points)
%                     img(points(i,1), points(i,2)) = 1; 
%                 end
%             end
figure;
            img = imfill(img,'holes');
            seD = strel('diamond',1);
            img = imerode(img,seD);
            img = imerode(img,seD);
            img = imcomplement(img);
            imwrite(img,'storage/topographic_map.png')
            imshow(img);
        end
        
        function [xPixel, yPixel] = getCirclePositon(image, color)
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
            for i=1:imageSize(1)
                for j=1:imageSize(2)
                    if (strcmp(color, 'blue'))
                        indicator1 = imageValues(i,j,3)/imageValues(i,j,1);
                        indicator2 = imageValues(i,j,3)/imageValues(i,j,2);
                        indicator3 = imageValues(i,j,3);
                        if (indicator1 > 2 && indicator2 > 2 && indicator3 > (100/255)) 
                            count = count + 1;
                            points(count, :) = [i j];
%                             for k1 = i:(i+15)
%                                 for k2 = j:(j+15)
%                                     image(k1,k2,1) = 255;
%                                     image(k1,k2,2) = 255;
%                                     image(k1,k2,3) = 255; 
%                                 end
%                             end
%                             image(i,j,1) = 255;
%                             image(i,j,2) = 255;
%                             image(i,j,3) = 255; 
                        end
                    elseif (strcmp(color, 'red'))
                        indicator1 = imageValues(i,j,1)/imageValues(i,j,2);
                        indicator2 = imageValues(i,j,1)/imageValues(i,j,3);
                        if (imageValues(i,j,1) > (180/255) && indicator1 > 2 && indicator2 > 2)
                            count = count + 1;
                            points(count, :) = [i j];
%                             for k1 = i:(i+15)
%                                 for k2 = j:(j+15)
%                                     image(k1,k2,1) = 255;
%                                     image(k1,k2,2) = 255;
%                                     image(k1,k2,3) = 255; 
%                                 end
%                             end
%                             image(i,j,1) = 255;
%                             image(i,j,2) = 255;
%                             image(i,j,3) = 255; 
                        end   
                    end
                end
            end

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
        end
    end
end

