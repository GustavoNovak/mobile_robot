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
        
        function parameters = calibrate(C, focusLength, thetaMax, chamberSize, cameraNumber, cameras)
            % Inform the calibration points here
            [calibrationPoints, pixelValuesCalibrationPoints, imageSize] = services.Images.getCalibrationPoints(cameraNumber, cameras, chamberSize);
            calibrationPoints = [-1390 -1450 0 ; 1390 -1450 0 ; -1390 1450 0 ; 1390 1450 0];
            if(cameraNumber == 1)
                pixelValuesCalibrationPoints = [103 547 ; 870 559 ; 255 5 ; 740 19]; 
            else
                pixelValuesCalibrationPoints = [117 10 ; -15 460 ; 557 3 ; 694 470]; 
            end

            numberPoints = size(calibrationPoints);
            numberPoints = numberPoints(1);
            
            if ~exist('u0','var') 
                [mu, mv, u0, v0] = C.getPixelsMappingModel(imageSize, cameraNumber);
            end

            % Get pixel values of calibrationPoint from the image
            xUnitSphere = [];
            length(pixelValuesCalibrationPoints)
            for i1 = 1:length(pixelValuesCalibrationPoints)
                xUnitSphere(i1, :) = [
                    pixelValuesCalibrationPoints(i1, 1) 
                    pixelValuesCalibrationPoints(i1, 2)
                    1
                ];
            end

            A = [];
            countA = 0;
            for i1 = 1:length(pixelValuesCalibrationPoints)
                if(calibrationPoints(i1, 3) == 0) 
                    xp = calibrationPoints(i1, 1);
                    yp = calibrationPoints(i1, 2);
                    zp = 1;

                    xus = xUnitSphere(i1, 1);
                    yus = xUnitSphere(i1, 2);
                    zus = xUnitSphere(i1, 3);

                    countA = countA + 1;
                    A(countA, :) = [0 0 0 -xp*zus -yp*zus -zp*zus xp*yus yp*yus zp*yus];
                    countA = countA + 1;
                    A(countA, :) = [xp*zus yp*zus zp*zus 0 0 0 -xp*xus -yp*xus -zp*xus];
                end 
            end
            [U,S,V] = svd(A);
            h = V(:, end);
            H = [h(1) h(2) h(3) ; h(4) h(5) h(6) ; h(7) h(8) h(9)];
            
            M = [mu 0 u0 ; 0 mv v0 ; 0 0 1];
            F = [focusLength 0 0 ; 0 focusLength 0 ; 0 0 1];
            B = inv(M*F)*H;
            lambda = sign(B(3, 3))/norm([B(1, 1) ; B(2, 1) ; B(3, 1)]);

            r1 = lambda*[B(1, 1) ; B(2, 1) ; B(3, 1)];
            r2 = lambda*[B(1, 2) ; B(2, 2) ; B(3, 2)];
            r3 = cross(r1, r2);

            Q = [r1 r2 r3];
            [U,S,V] = svd(Q);
            R = U*V';

            tUnique = lambda*[B(1, 3) ; B(2, 3) ; B(3, 3)];

            t = [
                tUnique(1)
                tUnique(2) 
                tUnique(3)
            ]
            angles = [
                -asin(R(3, 1)) 
                atan2((R(3, 2)/cos(-asin(R(3, 1)))),(R(3, 3)/cos(-asin(R(3, 1))))) 
                atan2((R(2, 1)/cos(-asin(R(3, 1)))),(R(1, 1)/cos(-asin(R(3, 1)))))
            ]

            gradientSize = 9;
            % Optimize
            for iterations = 1:800000
                finalGradient = zeros(1,gradientSize);
                errorValue = 0;
                for i1 = 1:length(pixelValuesCalibrationPoints)
                    X = [calibrationPoints(i1, 1) ; calibrationPoints(i1, 2) ; calibrationPoints(i1, 3)];
                    [gradient, singleErrorValue] = C.calculateGradient(X, angles, t, focusLength, [mu mv u0 v0], pixelValuesCalibrationPoints(i1, :));
                    finalGradient = finalGradient + gradient;
                    errorValue = errorValue + singleErrorValue;
                end 
                errorValue
                step = 1/5000000;

                for j=1:3
                    angles(j) = angles(j) - step*finalGradient(j);
                    t(j) = t(j) - step*finalGradient(j + 3);
                end
%                     focusLength = focusLength - step*finalGradient(7);
    %                 k1 = k1 - step*finalGradient(6*viewsLength+1);
    %                 k2 = k2 - step*finalGradient(6*viewsLength+2);
    %                 k3 = k3 - step*finalGradient(6*viewsLength+3);
    %                 k4 = k4 - step*finalGradient(6*viewsLength+4);
    %                 k5 = k5 - step*finalGradient(6*viewsLength+5);
                mu = mu - step*finalGradient(8);
                mv = mv - step*finalGradient(9);
    %                 u0 = u0 - step*finalGradient(6*viewsLength+8);
    %                 v0 = v0 - step*finalGradient(6*viewsLength+9);
            end
            [calibrationPointsNumber, n] = size(calibrationPoints);
            errorValue = (errorValue/calibrationPointsNumber)^(0.5);
            % Storing parameters
            warning('off', 'MATLAB:MKDIR:DirectoryExists');
            mkdir storage;
            fileName = fopen(strcat('storage/camera_calibration_', num2str(cameraNumber), '.dat'),'w');
            fileName2 = fopen(strcat('storage/input_cameras_calibration_', num2str(cameraNumber), '.dat'),'w');
            calibrationData = [
                angles(1) angles(2) angles(3) t(1) t(2) t(3) focusLength mu mv u0 v0 
            ];
            fprintf(fileName, '%f %f %f %f %f %f %f %f %f %d %d', calibrationData);
            fclose(fileName);
            
            C.parameters(cameraNumber, 1) = angles(1);
            C.parameters(cameraNumber, 2) = angles(2);
            C.parameters(cameraNumber, 3) = angles(3);
            C.parameters(cameraNumber, 4) = t(1);
            C.parameters(cameraNumber, 5) = t(2);
            C.parameters(cameraNumber, 6) = t(3);
            C.parameters(cameraNumber, 7) = focusLength;
            C.parameters(cameraNumber, 8) = mu;
            C.parameters(cameraNumber, 9) = mv;
            C.parameters(cameraNumber, 10) = u0;
            C.parameters(cameraNumber, 11) = v0;
            error = 0;
            for i1 = 1:length(pixelValuesCalibrationPoints)
                pixelPosition = [pixelValuesCalibrationPoints(i1, 1) pixelValuesCalibrationPoints(i1, 2)];
                realPosition = C.getRealPosition(cameraNumber, pixelPosition, 0) - [calibrationPoints(i1, 1) calibrationPoints(i1, 2) 0];
                error = error + norm(realPosition);
            end
            
            fprintf(fileName2, '%f %d %d %d %f', [ focusLength round(thetaMax*(360/pi)) chamberSize(1) chamberSize(2) error/i1 ]);
            fclose(fileName2);
            
            errorValue

            angles
            t
            parameters = [
                focusLength
                mu 
                mv 
                u0 
                v0
            ]
        end 
        
        function pixelPosition = calculatePosition(position)
            R = services.Math.getRotationMatrix(2.7949, 0.7192, -1.7821);
            pixelPosition = classes.Cameras.calculatePixelPosition(position, R, [60.8280 ; -35.0087 ; 314.4015], [13.7343 2.1443 0.5921 -0.3506 0.1317], [33.6277 30.5058 203.4397 179.9887])
        end
        
        function [k1, k2, k3, k4, k5] = fittingPolynomialModelToDesiredProjection(C, focusLength, thetaMax, projectionType)
            theta=0:0.1/180*pi:thetaMax;
            
            switch projectionType
                case 'equidistance'
                    k1 = focusLength;
                    k2 = 0; 
                case 'perspective'
                    persp=focusLength*tan(theta);
                    k=functions.polyfitoddlsq(theta,persp,9);
                    k1=k(9); k2=k(7); k3=k(5); k4=k(3); k5=k(1); 
                otherwise
                    error('Invalid Projection type')
            end
        end
        
        function [mu, mv, u0, v0] = getPixelsMappingModel(C, imageSize, cameraNumber)
            a = imageSize(1);
            b = imageSize(2);
            
            if(cameraNumber == 1)
                mu = 165;
                mv = 168;
            else
                mu = 141;
                mv = 146;
            end
            u0 = a/2;
            v0 = b/2;
        end
        
        function [x, y] = getSensorPosition(C, pixelPosition, mu, mv, u0, v0)
            T = [(1/mu) 0 ; 0 (1/mv)];
            centeredPixelPositions = [(pixelPosition(1) - u0) ; (pixelPosition(2) - v0)];
            
            aux = T*centeredPixelPositions;
            
            x = aux(1);
            y = aux(2);
        end
        
        function realPosition = getRealPosition(C, cameraNumber, m, z)
            lambda = 1;
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
            T = [R(1, 1) R(1, 2) t(1) ; R(2, 1) R(2, 2) t(2) ; R(3, 1) R(3, 2) t(3)];
            F = [focusLength 0 0 ; 0 focusLength 0 ; 0 0 1];
            M = [mu 0 u0 ; 0 mv v0 ; 0 0 1];
            I = M*F;

            error = 1;
            while (error > 0.00001)
                V = inv(T)*(inv(I)*[m(1) ; m(2) ; 1] - lambda*z*[R(1, 3) ; R(2, 3) ; R(3, 3)]);
                oldLambda = lambda;
                lambda = V(3);
                error = abs(lambda - oldLambda);
            end

            realPosition = [V(1)/V(3) V(2)/V(3) z];
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
        
        function pixelPosition = calculatePixelPosition(C, X, R, t, focusLength, M)
%             k1 = K(1);
%             k2 = K(2);
%             k3 = K(3);
%             k4 = K(4);
%             k5 = K(5);
            mu = M(1);
            mv = M(2);
            u0 = M(3);
            v0 = M(4);
            
            Xc = R*X + t;
            theta = atan2((Xc(1)^2 + Xc(2)^2)^0.5, Xc(3));
            phi = atan2(Xc(2), Xc(1));
            r = focusLength*tan(theta);
            sensorPosition = r*[cos(phi) ; sin(phi)];
            pixelPosition = [mu 0 ; 0 mv]*sensorPosition + [u0 ; v0];
        end
        
        function [gradient, neutralErrorValue] = calculateGradient(C, X, angles, t, focusLength, M, pixelValuesCalibrationPoint)
            % Parameters
            delta = 0.0001;
            
            theta = angles(1);
            psi = angles(2);
            phi = angles(3); 
            t = [
               t(1) ; 
               t(2) ; 
               t(3) ; 
            ];
            for i=1:6
                gradient(i) = 0;
            end
%             k1 = K(1);
%             k2 = K(2);
%             k3 = K(3);
%             k4 = K(4);
%             k5 = K(5);
            mu = M(1);
            mv = M(2);
            u0 = M(3);
            v0 = M(4); 
            
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            neutralErrorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            
            % psi
            theta = theta + delta;
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            theta = theta - delta;
            
            gradient(1) = (errorValue - neutralErrorValue)/delta;
            
            % theta
            psi = psi + delta;
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            psi = psi - delta;
            
            gradient(2) = (errorValue - neutralErrorValue)/delta;
            
            % phi
            phi = phi + delta;
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            phi = phi - delta;
            
            gradient(3) = (errorValue - neutralErrorValue)/delta;
            
            % t1
            t(1) = t(1) + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            t(1) = t(1) - delta;
            
            gradient(4) = (errorValue - neutralErrorValue)/delta;            
            % t2
            t(2) = t(2) + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            t(2) = t(2) - delta;
            
            gradient(5) = (errorValue - neutralErrorValue)/delta;  
            
            % t3
            t(3) = t(3) + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            t(3) = t(3) - delta;
            
            gradient(6) = (errorValue - neutralErrorValue)/delta;  
            
            % focusLength
            focusLength = focusLength + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            focusLength = focusLength - delta;
            
            gradient(7) = (errorValue - neutralErrorValue)/delta;
            
%             % k1
%             k1 = k1 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             k1 = k1 - delta;
%             
%             gradient(6*viewsLength + 1) = (errorValue - neutralErrorValue)/delta; 
%             
%             % k2
%             k2 = k2 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             k2 = k2 - delta;
%             
%             gradient(6*viewsLength + 2) = (errorValue - neutralErrorValue)/delta; 
%             
%             % k3
%             k3 = k3 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             k3 = k3 - delta;
%             
%             gradient(6*viewsLength + 3) = (errorValue - neutralErrorValue)/delta; 
%             
%             % k4
%             k4 = k4 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             k4 = k4 - delta;
%             
%             gradient(6*viewsLength + 4) = (errorValue - neutralErrorValue)/delta; 
%             
%             % k5
%             k5 = k5 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             k5 = k5 - delta;
%             
%             gradient(6*viewsLength + 5) = (errorValue - neutralErrorValue)/delta; 
%             
            % mu
            mu = mu + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            mu = mu - delta;
            
            gradient(8) = (errorValue - neutralErrorValue)/delta; 
            
            % mv
            mv = mv + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(1))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(2))^2;
            mv = mv - delta;
            
            gradient(9) = (errorValue - neutralErrorValue)/delta;   
%             
%             % u0
%             u0 = u0 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             u0 = u0 - delta;
%             
%             gradient(6*viewsLength + 8) = (errorValue - neutralErrorValue)/delta; 
%             
%             % v0
%             v0 = v0 + delta;
%             pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
%             errorValue = pixelPosition - [pixelValuesCalibrationPoint(2) ; pixelValuesCalibrationPoint(3)];
%             errorValue = norm(errorValue);
%             v0 = v0 - delta;
%             
%             gradient(6*viewsLength + 9) = (errorValue - neutralErrorValue)/delta;  
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
