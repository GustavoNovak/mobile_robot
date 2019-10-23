classdef Cameras
    properties
        parameters
    end
    
    methods
        function C = Cameras()
            try
                C.parameters = services.Storage.getCameraParameters();
            catch
            end
        end
        
        function [x, y, phi] = getRobotPosition(C, robotPosition, cameras, velocity, timer, countSettedVelocity, robot)
            x = 0;
            y = 0;
            phi = 0;
            
            count = 0;
            for k1 = 1:length(cameras)
                pixelPosition = C.getRealPixelPosition(k1, [robotPosition(1) ; robotPosition(2) ; 220]);
                pixelPosition(1) = floor(pixelPosition(1));
                pixelPosition(2) = floor(pixelPosition(2));
                
                img = getsnapshot(cameras(k1));

                [countRed, xRed, yRed, countSettedVelocity] = services.Images.getCirclePositon(img, 'red', pixelPosition, k1, velocity, timer, countSettedVelocity, robot);
                [countGreen, xGreen, yGreen, countSettedVelocity] = services.Images.getCirclePositon(img, 'green', pixelPosition, k1, velocity, timer, countSettedVelocity, robot);
                
                if (xRed > -10000 && xGreen > -10000 && k1 == 1)
                    verificationValue = countRed/countGreen;
                    if(verificationValue > 0.2 && verificationValue < 5)
                        realPositionRed = C.getRealPosition(k1, [xRed yRed], 235);
                        realPositionGreen = C.getRealPosition(k1, [xGreen yGreen], 235);

                        count = count + 1;
                        x = x + (realPositionRed(1) + realPositionGreen(1))/2;
                        y = y + (realPositionRed(2) + realPositionGreen(2))/2;

                        vectorPosition = [(realPositionRed(1) - realPositionGreen(1)) ; (realPositionRed(2) - realPositionGreen(2))];
                        vectorPosition = [0 1 ; -1 0] * vectorPosition;
                        
                      
                        phi = phi + atan2(vectorPosition(2), vectorPosition(1));
                        break;
                    end
                end
            end
            
            if(count > 0)
                x = x/count;
                y = y/count;
                phi = phi/count;
            else
                x = -999999;
                y = -999999;
                phi = -999999;
            end
        end
        
        function parameters = calibrate(C, chamberSize, cameras)
            % Get pixel valus and calibration points
%             [calibrationPoints, pixelValuesCalibrationPoints, imageSize] = services.Images.getCalibrationPoints(cameraNumber, cameras, chamberSize);
            calibrationPoints{1} = [-1390 -1450 0 ; 1390 -1450 0 ; -1390 1450 0 ; 1390 1450 0];
            calibrationPoints{2} = [-1390 -1450 0 ; 1390 -1450 0 ; -1390 1450 0 ; 1390 1450 0];
            pixelValuesCalibrationPoints{1} = [104 547 ; 870 561 ; 254 6 ; 739 21]; 
            pixelValuesCalibrationPoints{2} = [123 13 ; -7 463 ; 563 1 ; 694 470]; 

            numberPoints = size(calibrationPoints);
            numberPoints = numberPoints(1);
            
            % Get measured pixel values of calibrationPoint from the image
            if ~exist('u0','var')
                for cameraNumber = 1:length(cameras) 
                    if(cameraNumber == 1)
                        fx(cameraNumber) = 610.5;
                        fy(cameraNumber) = 621.6;
                        u0(cameraNumber) = 1024/2;
                        v0(cameraNumber) = 576/2;
                    else
                        imageSize = [640 480];
                        fx(cameraNumber) = 521.7;
                        fy(cameraNumber) = 540.2;
                        u0(cameraNumber) = 640/2;
                        v0(cameraNumber) = 480/2;
                    end
                end
            end

            % Get measured pixel values of calibrationPoint from the image
            measuredPixelValues = [];
            
            for cameraNumber = 1:length(cameras) 
                for i = 1:length(pixelValuesCalibrationPoints{1})
                    measuredPixelValues{cameraNumber}(i, :) = [
                        pixelValuesCalibrationPoints{cameraNumber}(i, 1) 
                        pixelValuesCalibrationPoints{cameraNumber}(i, 2)
                        1
                    ];
                end
                
                countA = 0;
                for i = 1:length(pixelValuesCalibrationPoints{1})
                    xp = calibrationPoints{cameraNumber}(i, 1);
                    yp = calibrationPoints{cameraNumber}(i, 2);
                    zp = 1;

                    xus = measuredPixelValues{cameraNumber}(i, 1);
                    yus = measuredPixelValues{cameraNumber}(i, 2);
                    zus = measuredPixelValues{cameraNumber}(i, 3);

                    countA = countA + 1;
                    A{cameraNumber}(countA, :) = [0 0 0 -xp*zus -yp*zus -zp*zus xp*yus yp*yus zp*yus];
                    countA = countA + 1;
                    A{cameraNumber}(countA, :) = [xp*zus yp*zus zp*zus 0 0 0 -xp*xus -yp*xus -zp*xus];
                end
                
                [U,S,V] = svd(A{cameraNumber});
                h = V(:, end);
                H = [h(1) h(2) h(3) ; h(4) h(5) h(6) ; h(7) h(8) h(9)];
                       
                M{cameraNumber} = [fx(cameraNumber) 0 u0(cameraNumber) ; 0 fy(cameraNumber) v0(cameraNumber) ; 0 0 1];
                B = inv(M{cameraNumber})*H;
                lambda = sign(B(3, 3))/norm([B(1, 1) ; B(2, 1) ; B(3, 1)]);

                r1 = lambda*[B(1, 1) ; B(2, 1) ; B(3, 1)];
                r2 = lambda*[B(1, 2) ; B(2, 2) ; B(3, 2)];
                r3 = cross(r1, r2);

                Q = [r1 r2 r3];
                [U,S,V] = svd(Q);
                R = U*V';
            

                tUnique = lambda*[B(1, 3) ; B(2, 3) ; B(3, 3)];

                t{cameraNumber} = [
                    tUnique(1)
                    tUnique(2) 
                    tUnique(3)
                ];
                angles{cameraNumber} = [
                    -asin(R(3, 1)) 
                    atan2((R(3, 2)/cos(-asin(R(3, 1)))),(R(3, 3)/cos(-asin(R(3, 1))))) 
                    atan2((R(2, 1)/cos(-asin(R(3, 1)))),(R(1, 1)/cos(-asin(R(3, 1)))))
                ];
            
                angles{cameraNumber}
                t{cameraNumber}
                M{cameraNumber}
                calibrationPoints{cameraNumber}
                pixelValuesCalibrationPoints{cameraNumber}
            end

            % Optimize
            step = 0.001;
            for iterations = 1:5000
                [gradient, errorValue] = C.calculateGradient(angles, t, M, calibrationPoints, pixelValuesCalibrationPoints);
                
                for cameraNumber = 1:length(cameras) 
                    for j=1:3
                        t{cameraNumber}(j) = t{cameraNumber}(j) - 100*step*gradient{cameraNumber}(j);
                        angles{cameraNumber}(j) = angles{cameraNumber}(j) - 0.00001*step*gradient{cameraNumber}(j + 3);
                    end
                    M{cameraNumber}(1, 1) = M{cameraNumber}(1, 1) - step*gradient{cameraNumber}(7);
                    M{cameraNumber}(2, 2) = M{cameraNumber}(2, 2) - step*gradient{cameraNumber}(8);
                end
            end
            
            for cameraNumber = 1:length(cameras) 
                t{cameraNumber}
                angles{cameraNumber}
                M{cameraNumber}
                
                % Storing parameters
                warning('off', 'MATLAB:MKDIR:DirectoryExists');
                mkdir storage;

                fileName = fopen(strcat('storage/camera_calibration_', num2str(cameraNumber), '.dat'),'w');
                fileName2 = fopen(strcat('storage/input_cameras_calibration_', num2str(cameraNumber), '.dat'),'w');
                calibrationData = [
                    angles{cameraNumber}(1) angles{cameraNumber}(2) angles{cameraNumber}(3) t{cameraNumber}(1) t{cameraNumber}(2) t{cameraNumber}(3) M{cameraNumber}(1, 1) M{cameraNumber}(2, 2) M{cameraNumber}(1, 3) M{cameraNumber}(2, 3) 
                ];
                fprintf(fileName, '%f %f %f %f %f %f %f %f %d %d', calibrationData);
                fclose(fileName);
                
                realError = 0;
                for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                    m = [pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 1) ; pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 2)];
                    realPosition = C.getRealPosition(m, 0, angles{cameraNumber}, t{cameraNumber}, M{cameraNumber});
                    realError = realError + (realPosition(1) - calibrationPoints{cameraNumber}(pointNumber, 1))^2 + + (realPosition(2) - calibrationPoints{cameraNumber}(pointNumber, 2))^2;
                end

                fprintf(fileName2, '%d %d %f', [ chamberSize(1) chamberSize(2) (realError/pointNumber)^0.5 ]);
                fclose(fileName2);       
                
                C.parameters(cameraNumber, 1) = angles{cameraNumber}(1);
                C.parameters(cameraNumber, 2) = angles{cameraNumber}(2);
                C.parameters(cameraNumber, 3) = angles{cameraNumber}(3);
                C.parameters(cameraNumber, 4) = t{cameraNumber}(1);
                C.parameters(cameraNumber, 5) = t{cameraNumber}(2);
                C.parameters(cameraNumber, 6) = t{cameraNumber}(3);
                C.parameters(cameraNumber, 7) = M{cameraNumber}(1, 1);
                C.parameters(cameraNumber, 8) = M{cameraNumber}(2, 2);
                C.parameters(cameraNumber, 9) = M{cameraNumber}(1, 3);
                C.parameters(cameraNumber, 10) = M{cameraNumber}(2, 3);
            end
        end 
        
        function pixelPosition = calculatePosition(position)
            R = services.Math.getRotationMatrix(2.7949, 0.7192, -1.7821);
            pixelPosition = classes.Cameras.calculatePixelPosition(position, R, [60.8280 ; -35.0087 ; 314.4015], [13.7343 2.1443 0.5921 -0.3506 0.1317], [33.6277 30.5058 203.4397 179.9887])
        end
        
        function [x, y] = getSensorPosition(C, pixelPosition, mu, mv, u0, v0)
            T = [(1/mu) 0 ; 0 (1/mv)];
            centeredPixelPositions = [(pixelPosition(1) - u0) ; (pixelPosition(2) - v0)];
            
            aux = T*centeredPixelPositions;
            
            x = aux(1);
            y = aux(2);
        end
        
        function realPosition = getRealPositionWithStoredParameters(C, cameraNumber, m, z)
            angles = [C.parameters(cameraNumber, 1) C.parameters(cameraNumber, 2) C.parameters(cameraNumber, 3)];
            t = [C.parameters(cameraNumber, 4) C.parameters(cameraNumber, 5) C.parameters(cameraNumber, 6)];
            M = [
                C.parameters(cameraNumber, 7) 0 C.parameters(cameraNumber, 9) ;  
                0 C.parameters(cameraNumber, 8) C.parameters(cameraNumber, 10) ; 
                0 0 1 
            ];
        
            realPosition = C.getRealPosition(m, z, angles, t, M);
        end
        
        function realPosition = getRealPosition(C, m, z, angles, t, M)
            R = services.Math.getRotationMatrix(angles(1), angles(2), angles(3));
            
            M = [M(1, 1) M(1, 2) M(1, 3) 0 ; M(2, 1) M(2, 2) M(2, 3) 0; M(3, 1) M(3, 2) M(3, 3) 0];
            T = [R(1, 1) R(1, 2) R(1, 3) t(1) ; R(2, 1) R(2, 2) R(2, 3) t(2) ; R(3, 1) R(3, 2) R(3, 3) t(3) ; 0 0 0 1];
            C = M*T;
            
            A = [(C(1, 1) - m(1)*C(3, 1)) (C(1, 2) - m(1)*C(3, 2)) ; (C(2, 1) - m(2)*C(3, 1)) (C(2, 2) - m(2)*C(3, 2))];
            b = [(-(C(1, 3)*z + C(1, 4)) + m(1)*(C(3, 3)*z + C(3,4))) ; (-(C(2, 3)*z + C(2, 4)) + m(2)*(C(3, 3)*z + C(3,4)))];
            
            realPosition = A\b;
            
            realPosition = [realPosition(1) realPosition(2) z];
        end
        
        function pixelPosition = getRealPixelPosition(C, cameraNumber, X)
            theta = C.parameters(cameraNumber, 1);
            psi = C.parameters(cameraNumber, 2);
            phi = C.parameters(cameraNumber, 3);
            t = [C.parameters(cameraNumber, 4); C.parameters(cameraNumber, 5); C.parameters(cameraNumber, 6)];
            focusLength = C.parameters(cameraNumber, 7);
            mu = C.parameters(cameraNumber, 8);
            mv = C.parameters(cameraNumber, 9);
            u0 = C.parameters(cameraNumber, 10);
            v0 = C.parameters(cameraNumber, 11);
            
            R = services.Math.getRotationMatrix(psi, theta, phi); 
            Xc = R*X + t;
            theta = atan2((Xc(1)^2 + Xc(2)^2)^0.5, Xc(3));
            phi = atan2(Xc(2), Xc(1));
            r = focusLength*tan(theta);
            sensorPosition = r*[cos(phi) ; sin(phi)];
            pixelPosition = [mu 0 ; 0 mv]*sensorPosition + [u0 ; v0];        
        end
        
        function pixelPosition = calculatePixelPosition(C, X, angles, t, M) 
            R = services.Math.getRotationMatrix(angles(1), angles(2), angles(3));
            
            Xc = R*X + t;
            pixelPosition = M*Xc;
           
            pixelPosition = [pixelPosition(1) pixelPosition(2)]/pixelPosition(3);
        end
        
        function [gradient, neutralErrorValue] = calculateGradient(C, angles, t, M, calibrationPoints, pixelValuesCalibrationPoints)
            % Parameters
            delta = 0.0001;
            sigularityfactor = 1;
            
            for cameraNumber = 1:length(pixelValuesCalibrationPoints)
                p{cameraNumber} = [t{cameraNumber}(1) t{cameraNumber}(2) t{cameraNumber}(3) angles{cameraNumber}(1) angles{cameraNumber}(2) angles{cameraNumber}(3) M{cameraNumber}(1, 1) M{cameraNumber}(2, 2)];
            end
            
            % Calculating Neutral Error
            neutralErrorMeasurementValue = 0;
            
            % Measurement Part
            for cameraNumber = 1:length(pixelValuesCalibrationPoints)
                for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                    X = [calibrationPoints{cameraNumber}(pointNumber, 1) ; calibrationPoints{cameraNumber}(pointNumber, 2) ; calibrationPoints{cameraNumber}(pointNumber, 3)];
                    angles = [p{cameraNumber}(4) p{cameraNumber}(5) p{cameraNumber}(6)];
                    t = [p{cameraNumber}(1) ; p{cameraNumber}(2) ; p{cameraNumber}(3)];
                    intrinsicMatrix = [p{cameraNumber}(7) 0 M{cameraNumber}(1, 3) ; 0 p{cameraNumber}(8) M{cameraNumber}(2, 3) ; 0 0 1];
                    pixelPosition = C.calculatePixelPosition(X, angles, t, intrinsicMatrix);
                    neutralErrorMeasurementValue = neutralErrorMeasurementValue + (pixelPosition(1) - pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 2))^2;
                end
                
                gradient{cameraNumber} = zeros(1, 8);
            end

            % Singularity Part
            neutralErrorSingularityValue = 0;

            pointPositions = [];
            for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                pointPositions{pointNumber} = [0 0];
            end
            
            for cameraNumber = 1:length(pixelValuesCalibrationPoints)
                for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                    m = [pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 1) ; pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 2)];
                    angles = [p{cameraNumber}(4) p{cameraNumber}(5) p{cameraNumber}(6)];
                    t = [p{cameraNumber}(1) ; p{cameraNumber}(2) ; p{cameraNumber}(3)];
                    intrinsicMatrix = [p{cameraNumber}(7) 0 M{cameraNumber}(1, 3) ; 0 p{cameraNumber}(8) M{cameraNumber}(2, 3) ; 0 0 1];
                    realPosition = C.getRealPosition(m, 0, angles, t, intrinsicMatrix);
                    pointPositions{pointNumber}(1) = pointPositions{pointNumber}(1) + realPosition(1);
                    pointPositions{pointNumber}(2) = pointPositions{pointNumber}(2) + realPosition(2);
                end
            end    
            
            for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                pointPositions{pointNumber} = pointPositions{pointNumber}/length(pixelValuesCalibrationPoints);
            end
            
            for cameraNumber = 1:length(pixelValuesCalibrationPoints)
                for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                    m = [pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 1) ; pixelValuesCalibrationPoints{cameraNumber}(pointNumber, 2)];
                    angles = [p{cameraNumber}(4) p{cameraNumber}(5) p{cameraNumber}(6)];
                    t = [p{cameraNumber}(1) ; p{cameraNumber}(2) ; p{cameraNumber}(3)];
                    intrinsicMatrix = [p{cameraNumber}(7) 0 M{cameraNumber}(1, 3) ; 0 p{cameraNumber}(8) M{cameraNumber}(2, 3) ; 0 0 1];
                    realPosition = C.getRealPosition(m, 0, angles, t, intrinsicMatrix);
                    neutralErrorSingularityValue = neutralErrorSingularityValue + (realPosition(1) - pointPositions{pointNumber}(1))^2 + (realPosition(2) - pointPositions{pointNumber}(2))^2;
                end
            end
            neutralErrorMeasurementValue         
            neutralErrorSingularityValue
            neutralErrorValue = neutralErrorMeasurementValue + sigularityfactor*neutralErrorSingularityValue;
            
            % Calculating Gradient
            for parameterId = 1:8
                for cameraNumber = 1:length(pixelValuesCalibrationPoints)
                    p{cameraNumber}(parameterId) = p{cameraNumber}(parameterId) + delta;
                    
                    % Measurement Part 
                    errorMeasurementValue = 0;
                    for cameraNumber2 = 1:length(pixelValuesCalibrationPoints)
                        for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                            X = [calibrationPoints{cameraNumber2}(pointNumber, 1) ; calibrationPoints{cameraNumber2}(pointNumber, 2) ; calibrationPoints{cameraNumber2}(pointNumber, 3)];
                            t = [p{cameraNumber2}(1) ; p{cameraNumber2}(2) ; p{cameraNumber2}(3)];
                            angles = [p{cameraNumber2}(4) p{cameraNumber2}(5) p{cameraNumber2}(6)];
                            intrinsicMatrix = [p{cameraNumber2}(7) 0 M{cameraNumber2}(1, 3) ; 0 p{cameraNumber2}(8) M{cameraNumber2}(2, 3) ; 0 0 1];
                            pixelPosition = C.calculatePixelPosition(X, angles, t, intrinsicMatrix);
                            errorMeasurementValue = errorMeasurementValue + (pixelPosition(1) - pixelValuesCalibrationPoints{cameraNumber2}(pointNumber, 1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoints{cameraNumber2}(pointNumber, 2))^2;
                        end
                    end
                    
                    % Singularity Part
                    errorSingularityValue = 0;

                    pointPositions = [];
                    for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                        pointPositions{pointNumber} = [0 0];
                    end

                    for cameraNumber2 = 1:length(pixelValuesCalibrationPoints)
                        for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                            m = [pixelValuesCalibrationPoints{cameraNumber2}(pointNumber, 1) ; pixelValuesCalibrationPoints{cameraNumber2}(pointNumber, 2)];
                            angles = [p{cameraNumber2}(4) p{cameraNumber2}(5) p{cameraNumber2}(6)];
                            t = [p{cameraNumber2}(1) ; p{cameraNumber2}(2) ; p{cameraNumber2}(3)];
                            intrinsicMatrix = [p{cameraNumber2}(7) 0 M{cameraNumber2}(1, 3) ; 0 p{cameraNumber2}(8) M{cameraNumber2}(2, 3) ; 0 0 1];
                            realPosition = C.getRealPosition(m, 0, angles, t, intrinsicMatrix);
                            pointPositions{pointNumber}(1) = pointPositions{pointNumber}(1) + realPosition(1);
                            pointPositions{pointNumber}(2) = pointPositions{pointNumber}(2) + realPosition(2);
                        end
                    end    

                    for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                        pointPositions{pointNumber} = pointPositions{pointNumber}/length(pixelValuesCalibrationPoints);
                    end

                    for cameraNumber2 = 1:length(pixelValuesCalibrationPoints)
                        for pointNumber = 1:length(pixelValuesCalibrationPoints{1})
                            m = [pixelValuesCalibrationPoints{cameraNumber2}(pointNumber, 1) ; pixelValuesCalibrationPoints{cameraNumber2}(pointNumber, 2)];
                            angles = [p{cameraNumber2}(4) p{cameraNumber2}(5) p{cameraNumber2}(6)];
                            t = [p{cameraNumber2}(1) ; p{cameraNumber2}(2) ; p{cameraNumber2}(3)];
                            intrinsicMatrix = [p{cameraNumber2}(7) 0 M{cameraNumber2}(1, 3) ; 0 p{cameraNumber2}(8) M{cameraNumber2}(2, 3) ; 0 0 1];
                            realPosition = C.getRealPosition(m, 0, angles, t, intrinsicMatrix);
                            errorSingularityValue = errorSingularityValue + (realPosition(1) - pointPositions{pointNumber}(1))^2 + (realPosition(2) - pointPositions{pointNumber}(2))^2;
                        end
                    end

                    errorValue = errorMeasurementValue + sigularityfactor*errorSingularityValue;
                    
                    p{cameraNumber}(parameterId) = p{cameraNumber}(parameterId) - delta;
                    
                    gradient{cameraNumber}(parameterId) = (errorValue - neutralErrorValue)/delta;
                end
            end
        end
        
        function [realTopographicMap, topographicMap] = generateTopographicMap(C, img, chamberSize, objectNumber, cameraNumber)
            sizeImage = size(img);
            img = imgaussfilt(img,2);
            img = rgb2gray(img);
            img = edge(img, 'Sobel');
            realTopographicMap = zeros(chamberSize(2), chamberSize(1));
            for i=1:chamberSize(2)
                percentage = 50*i/chamberSize(2)
                for j=1:chamberSize(1)
                    floorPixelPosition = C.getRealPixelPosition(cameraNumber, [j - chamberSize(1)/2 ; i - chamberSize(2)/2 ; 0]);
                    floorPixelPosition = round(floorPixelPosition);
                    if((floorPixelPosition(2) >= 1 && floorPixelPosition(1) >= 1) && (floorPixelPosition(2) <= sizeImage(1) && floorPixelPosition(1) <= sizeImage(2)))
                        realTopographicMap(i,j) = img(floorPixelPosition(2), floorPixelPosition(1)); 
                    end
                end            
            end
            figure;
            imshow(realTopographicMap);
            impixelinfo;

            se90 = strel('line',40,90);
            se0 = strel('line',40,0);
            realTopographicMap = imdilate(realTopographicMap ,[se90 se0]);
            
            figure;
            imshow(realTopographicMap);
            impixelinfo;
            
            realTopographicMap = bwareafilt(im2bw(realTopographicMap), objectNumber);
            realTopographicMap = imcomplement(realTopographicMap); 
            
            figure;
            imshow(realTopographicMap);
            impixelinfo;
            
            topographicMap = zeros(chamberSize(2), chamberSize(1));            
            for i=300:chamberSize(2) - 300
                percentage = 50 + 50*(i - 300)/(chamberSize(2) - 600)
                for j=300:chamberSize(1) - 300
                    if(realTopographicMap(i,j) == 0)
                        for theta = 0:(pi/50):2*pi
                            circlePosition = [i j] + 185*[cos(theta) sin(theta)];
                            circlePosition(1) = floor(circlePosition(1));
                            circlePosition(2) = floor(circlePosition(2));
                            if((circlePosition(1) >= 1 && circlePosition(2) >= 1) && (circlePosition(1) <= chamberSize(2) && circlePosition(2) <= chamberSize(1)))
                                topographicMap(circlePosition(1), circlePosition(2)) = 1;                      
                            end
                        end  
                    end    
                end       
            end

            topographicMap = imfill(topographicMap,'holes');
            topographicMap = imcomplement(topographicMap);
            
            for i=1:chamberSize(2)
                for j=1:chamberSize(1)
                    if(i <= 300 || i >= (chamberSize(2) - 300) || j <= 300 || j >= (chamberSize(1) - 300))
                        topographicMap(i, j) = 0; 
                    end
                end     
            end
            
            realTopographicMap = imfill(imcomplement(realTopographicMap),'holes');   
            realTopographicMap = imcomplement(realTopographicMap);   
            
            figure;
            imshow(realTopographicMap);
            impixelinfo;
            figure;
            imshow(topographicMap);
            impixelinfo;
            
%             figure;
%             imshow(topographicMap);
%             impixelinfo;
%             img = rgb2gray(img);
%             figure;
%             imshow(img);           
%             BW1 = edge(img,'sobel');
%             figure;
%             imshow(BW1);
%             se90 = strel('line',10,90);
%             se0 = strel('line',10,0);
%             BWsdil = imdilate(BW1,[se90 se0]);
%             BWdfill = imfill(BWsdil,'holes');
%             img = bwareafilt(BWdfill, 3);
%             imageSize = size(img);
%             
%             img = imcomplement(img);
%             
%             topographicMap = zeros(chamberSize(2), chamberSize(1));
%             for i=1:chamberSize(1)
%                 porcentage = 100*i/chamberSize(1)
%                 for j=1:chamberSize(2)
%                     floorPixelPosition = C.getRealPixelPosition(cameraNumber, [j - chamberSize(2)/2 ; i - chamberSize(1)/2 ; 0]);
%                     floorPixelPosition = round(floorPixelPosition);
%                     pixelPosition = C.getRealPixelPosition(cameraNumber, [j - chamberSize(2)/2 ; i - chamberSize(1)/2 ; 220]);
%                     pixelPosition = round(pixelPosition);
%                     if((pixelPosition(2) < 1 || pixelPosition(1) < 1) || (pixelPosition(2) > imageSize(1) || pixelPosition(1) > imageSize(2)))
%                         topographicMap(i, j) = 0;
%                     else
%                         if(img(floorPixelPosition(2), floorPixelPosition(1)) == 1)
%                             topographicMap(i, j) = 1;
%                         else
%                             topographicMap(i, j) = 0;
%                         end
%                     end
%                 end            
%             end
        end
        
        function cameras = getCameras(C)
            camera1 = videoinput('winvideo', 'Logitech Webcam C930e');
            camera2 = videoinput('winvideo', 'Logitech Webcam Pro 9000');
            triggerconfig(camera1, 'manual');
            triggerconfig(camera2, 'manual');
            src = getselectedsource(camera1);
            src.FocusMode = 'manual';
%             src.Exposure = -6;
            src = getselectedsource(camera2);
            src.FocusMode = 'manual';
            start(camera1);
            start(camera2);
            
            cameras = [camera1 camera2];
        end
    end
end
