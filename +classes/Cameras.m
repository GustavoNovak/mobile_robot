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
        
        function [x, y, phi] = getRobotPosition(C, pixelPosition, cameras)
            pixelPosition(1) = floor(pixelPosition(1));
            pixelPosition(2) = floor(pixelPosition(2));
            for k1 = 1:1
                tic
                img = getsnapshot(cameras(2));
%                 figure;
%                 imshow(img);
%                 impixelinfo;
%                 hold on;
                [xRed, yRed] = services.Images.getCirclePositon(img, 'red', pixelPosition);
                [xBlue, yBlue] = services.Images.getCirclePositon(img, 'blue', pixelPosition);
                
                realPositionRed = C.getRealPosition([xRed yRed], 220);
                realPositionBlue = C.getRealPosition([xBlue yBlue], 220);
                
                if (xRed ~= -999999 && xBlue ~= -999999)
                    x = (realPositionRed(1) + realPositionBlue(1))/2;
                    y = (realPositionRed(2) + realPositionBlue(2))/2;

                    vectorPosition = [(realPositionRed(1) - realPositionBlue(1)) (realPositionRed(2) - realPositionBlue(2))];

                    phi = atan2(vectorPosition(2), vectorPosition(1));
                else
                    x = -999999;
                    y = -999999;
                    phi = -999999;
                end
                t = toc
            end
        end
        
        function parameters = calibrate(C, focusLength, thetaMax, chamberSize)
            % Inputs
                % Inform the calibration points here
            calibrationPoints = [0 -1450 0 ; 0 1450 0 ; -1390 1450 0 ; 1390 1450 0 ; -1390 0 0 ; 0 0 0 ; 1390 0 0 ];
            %                  calibrationPoints = [-1000 -1350 0 ; 0 -1350 0 ; 1000 -1350 0 ; 0 1350 0 ; -1000 1350 0 ; 1000 1350 0 ; -900 -1250 200 ; 0 -1250 200 ; 900 -1250 200 ; 0 1250 200 ; -900 1250 100 ; 900 1250 100 ; -800 -1150 100 ; 0 -1150 0 ; 800 -1150 0 ; 0 1150 0 ; -800 1150 0 ; 800 1150 0 ; -700 -1050 0 ; 0 -1050 0 ; 700 -1050 0 ; 0 1050 0 ; -700 1050 0 ; 700 1050 0 ; -600 -950 0 ; 0 -950 0 ; 600 -950 0 ; 0 950 0 ; -600 950 0 ; 600 950 0 ; -500 -850 0 ; 0 -850 0 ; 500 -850 0 ; 0 850 0 ; -500 850 0 ; 500 850 0 ; -400 -750 0 ; 0 -750 0 ; 400 -750 0 ; 0 750 0 ; -400 750 0 ; 400 750 0 ; -300 -650 0 ; 0 -650 0 ; 300 -650 0 ; 0 650 0 ; -300 650 0 ; 300 650 0 ; -200 -550 0 ; 0 -550 0 ; 200 -550 0 ; 0 550 0 ; -200 550 0 ; 200 550 0 ; -100 -450 0 ; 0 -450 0 ; 100 -450 0 ; 0 450 0 ; -100 450 0 ; 100 450 0 ; 0 -350 0 ; 0 -350 0 ; 0 -350 0 ; 0 350 0 ; 0 350 0 ; 0 350 0 ; 100 -250 0 ; 0 -250 0 ; -100 -250 0 ; 0 250 0 ; 100 250 0 ; -100 250 0];
                % Inform the views here
%                 urlViews = {
%                    'storage/calibration_images/multiple_images/image_calibration_1.JPG'   
%                 };
%             'storage/calibration_images/multiple_images/image_calibration_2.JPG' 
%             'storage/calibration_images/multiple_images/image_calibration_3.JPG' 
        
            % End Inputs 
            
            numberPoints = size(calibrationPoints);
            numberPoints = numberPoints(1);
            
%             viewsLength = length(urlViews);
           viewsLength = 1;
%              pixelValuesCalibrationPoints = services.Simulator.calculatePointsInImage([-1000 -1350 0 ; 0 -1350 0 ; 1000 -1350 0 ; 0 1350 0 ; -1000 1350 0 ; 1000 1350 0 ; -900 -1250 200 ; 0 -1250 200 ; 900 -1250 200 ; 0 1250 200 ; -900 1250 100 ; 900 1250 100 ; -800 -1150 100 ; 0 -1150 0 ; 800 -1150 0 ; 0 1150 0 ; -800 1150 0 ; 800 1150 0 ; -700 -1050 0 ; 0 -1050 0 ; 700 -1050 0 ; 0 1050 0 ; -700 1050 0 ; 700 1050 0 ; -600 -950 0 ; 0 -950 0 ; 600 -950 0 ; 0 950 0 ; -600 950 0 ; 600 950 0 ; -500 -850 0 ; 0 -850 0 ; 500 -850 0 ; 0 850 0 ; -500 850 0 ; 500 850 0 ; -400 -750 0 ; 0 -750 0 ; 400 -750 0 ; 0 750 0 ; -400 750 0 ; 400 750 0 ; -300 -650 0 ; 0 -650 0 ; 300 -650 0 ; 0 650 0 ; -300 650 0 ; 300 650 0 ; -200 -550 0 ; 0 -550 0 ; 200 -550 0 ; 0 550 0 ; -200 550 0 ; 200 550 0 ; -100 -450 0 ; 0 -450 0 ; 100 -450 0 ; 0 450 0 ; -100 450 0 ; 100 450 0 ; 0 -350 0 ; 0 -350 0 ; 0 -350 0 ; 0 350 0 ; 0 350 0 ; 0 350 0 ; 100 -250 0 ; 0 -250 0 ; -100 -250 0 ; 0 250 0 ; 100 250 0 ; -100 250 0], 3.7, [80.5 60], [320 240]);
            
           
            pixelValuesCalibrationPoints = [1 329 476 ; 1 331 2 ; 1 126 5 ; 1 542 0 ; 1 75 179 ; 1 330 180 ; 1 593 180];
%             [calibrationPoints, pixelValuesCalibrationPoints] = services.Images.getCalibrationPoints(); 
            
%             pixelValuesCalibrationPoints = [
%                 1 208 133 ; 1 127 88 ; 1 98 193 ; 1 188 235 ; 1 199 185 ; 1 169 111 ; 1 113 139 ; 1 145 215 ; 1 158 160 ; 1 183 39 ; 1 268 91 ; 
%                 2 229 167 ; 2 260 81 ; 2 170 28 ; 2 134 121 ; 2 184 144 ; 2 244 124 ; 2 217 55 ; 2 152 73 ; 2 201 100 ; 2 322 78 ; 2 290 173 ; 
%                 3 92 78 ; 3 78 197 ; 3 232 211 ; 3 236 95 ; 3 166 87 ; 3 85 136 ; 3 156 204 ; 3 234 151 ; 3 161 146 ; 3 66 159 ; 3 84 17
%             ];
            
            countViews = 0;
            for i = 1:viewsLength
                countViews = countViews + 1;
%                 views{i} = imread(urlViews{i});
%                 imshow(views{i});
%                 impixelinfo;
                if ~exist('u0','var') 
%                     [k1, k2, k3, k4, k5] = C.fittingPolynomialModelToDesiredProjection(focusLength, thetaMax, 'perspective');
%                   % testing fitting
%                   thetaTest = linspace(0, thetaMax, 25);
%                   realFunction = focusLength*tan(thetaTest);
%                   aproxFunction = k1*thetaTest + k2*(thetaTest.^3) + k3*(thetaTest.^5) + k4*(thetaTest.^7) + + k5*(thetaTest.^9);
%                   plot(thetaTest, realFunction, thetaTest, aproxFunction);
%                   % ---

%                     rMax = k1*thetaMax + k2*(thetaMax^3) + k3*(thetaMax^5) + k4*(thetaMax^7) + k5*(thetaMax^9)
                    rMax = focusLength*tan(thetaMax);
%                     [mu, mv, u0, v0] = C.getPixelsMappingModel(rMax, views{i});
                    [mu, mv, u0, v0] = C.getPixelsMappingModel(rMax, []);
                end
                
                % Get pixel values of calibrationPoint from the image
                xUnitSphere = [];
                length(pixelValuesCalibrationPoints)
                for i1 = 1:length(pixelValuesCalibrationPoints)
                    if (pixelValuesCalibrationPoints(i1, 1) == i)
                        [x, y] = C.getSensorPosition(pixelValuesCalibrationPoints(i1, :), mu, mv, u0, v0);
                        [r, phi] = services.Math.getPolarCoordinates([x y]);
                        theta = atan2(r,focusLength);
%                         thetas = roots([k5 0 k4 0 k3 0 k2 0 k1 -r]);
%                         theta = 'empty';
%                         for k=1:length(thetas)
%                             if (imag(thetas(k)) == 0)
%                                 theta = thetas(k);
%                             end
%                         end
%                         if (theta == 'empty')
%                             error('Any real real theta was found, please change the code to consider this case');
%                         end

                        xUnitSphere(i1, :) = [
                            pixelValuesCalibrationPoints(i1, 2) 
                            pixelValuesCalibrationPoints(i1, 3)
                            1
                        ];
                    end
                end
                
                A = [];
                countA = 0;
                firstRegister = 0;
                for i1 = 1:length(pixelValuesCalibrationPoints)
                    if (pixelValuesCalibrationPoints(i1, 1) == i)
                        if (firstRegister == 0)
                            firstRegister = i;
                        end
                        iCalibration = (i1-numberPoints*(firstRegister-1));
                        
                        if(calibrationPoints(iCalibration, 3) == 0) 
                            xp = calibrationPoints(iCalibration, 1);
                            yp = calibrationPoints(iCalibration, 2);
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
                end
                [U,S,V] = svd(A);
                h = V(:, end);
                H = [h(1) h(2) h(3) ; h(4) h(5) h(6) ; h(7) h(8) h(9)];
                 
%                 for i1 = 1:length(pixelValuesCalibrationPoints)
%                     xp = calibrationPoints(i1, 1);
%                     yp = calibrationPoints(i1, 2);
%                     zp = 1;
%                     
%                     xus = xUnitSphere(i1, 1);
%                     yus = xUnitSphere(i1, 2);
%                     zus = xUnitSphere(i1, 3);
%                     aux = H*[xp ; yp ; zp];
%                     xCalculatedUnitSphere(i1, :) = aux / norm(aux);
%                     
%                     if (sign(xCalculatedUnitSphere(i1, 3)) ~= sign(zus))
%                         xCalculatedUnitSphere(i1, :) = -xCalculatedUnitSphere(i1, :);
%                     end
%                 end
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
                
                t(countViews, :) = [
                    tUnique(1)
                    tUnique(2) 
                    tUnique(3)
                ]
                angles(countViews, :) = [
                    -asin(R(3, 1)) 
                    atan2((R(3, 2)/cos(theta)),(R(3, 3)/cos(theta))) 
                    atan2((R(2, 1)/cos(theta)),(R(1, 1)/cos(theta)))
                ]
            end
            gradientSize = 3 + 6*viewsLength;
            % Optimize
            for iterations = 1:100000
                finalGradient = zeros(1,gradientSize);
                errorValue = 0;
                for i1 = 1:length(pixelValuesCalibrationPoints)
                    aux = i1/numberPoints;
                    if (floor(aux) > 0)
                        if (aux ~= floor(aux))
                            iCalibration = (i1-numberPoints*floor(aux));
                        else
                            iCalibration = (i1-numberPoints*(floor(aux)-1));
                        end
                    else
                        iCalibration = i1;
                    end
                    X = [calibrationPoints(iCalibration, 1) ; calibrationPoints(iCalibration, 2) ; calibrationPoints(iCalibration, 3)];
                    [gradient, singleErrorValue] = C.calculateGradient(X, angles, t, focusLength, [mu mv u0 v0], pixelValuesCalibrationPoints(i1, :), viewsLength);
                    finalGradient = finalGradient + gradient;
                    errorValue = errorValue + singleErrorValue;
                end 
                step = 1/1000000;

                for i=1:viewsLength
                    for j=1:3
                        angles(i, j) = angles(i, j) - step*finalGradient(3*(i-1)+j);
                        t(i, j) = t(i, j) - step*finalGradient(3*(viewsLength+i-1)+j);
                    end
                end
%                 focusLength = focusLength - step*finalGradient(6*viewsLength+1);
%                 k1 = k1 - step*finalGradient(6*viewsLength+1);
%                 k2 = k2 - step*finalGradient(6*viewsLength+2);
%                 k3 = k3 - step*finalGradient(6*viewsLength+3);
%                 k4 = k4 - step*finalGradient(6*viewsLength+4);
%                 k5 = k5 - step*finalGradient(6*viewsLength+5);
                mu = mu - step*finalGradient(6*viewsLength+2);
                mv = mv - step*finalGradient(6*viewsLength+3);
%                 u0 = u0 - step*finalGradient(6*viewsLength+8);
%                 v0 = v0 - step*finalGradient(6*viewsLength+9);
            end
            
            [calibrationPointsNumber, n] = size(calibrationPoints);
            errorValue = (errorValue/calibrationPointsNumber)^(0.5);
            % Storing parameters
            warning('off', 'MATLAB:MKDIR:DirectoryExists');
            mkdir storage;
            fileName = fopen('storage/camera_calibration.dat','w');
            fileName2 = fopen('storage/input_cameras_calibration.dat','w');
            calibrationData = [
                angles(1, 1) angles(1, 2) angles(1, 3) t(1, 1) t(1, 2) t(1, 3) focusLength mu mv u0 v0 
            ];
            fprintf(fileName, '%f %f %f %f %f %f %f %f %f %d %d', calibrationData);
            fclose(fileName);
            
            C.parameters(1) = angles(1, 1);
            C.parameters(2) = angles(1, 2);
            C.parameters(3) = angles(1, 3);
            C.parameters(4) = t(1, 1);
            C.parameters(5) = t(1, 2);
            C.parameters(6) = t(1, 3);
            C.parameters(7) = focusLength;
            C.parameters(8) = mu;
            C.parameters(9) = mv;
            C.parameters(10) = u0;
            C.parameters(11) = v0;
            error = 0;
            for i1 = 1:length(pixelValuesCalibrationPoints)
                pixelPosition = [pixelValuesCalibrationPoints(i1, 2) pixelValuesCalibrationPoints(i1, 3)];
                realPosition = C.getRealPosition(pixelPosition, 0) - [calibrationPoints(i1, 1) calibrationPoints(i1, 2) 0];
                error = error + norm(realPosition);
            end
            
            fprintf(fileName2, '%f %d %d %d %f', [ focusLength round(thetaMax*(360/pi)) chamberSize(1) chamberSize(2) error/i1 ]);
            fclose(fileName2);
            
            angles
            t
            parameters = [
                focusLength
                mu 
                mv 
                u0 
                v0
            ];
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
        
        function [mu, mv, u0, v0] = getPixelsMappingModel(C, rMax, view)
%             sizeView = size(view);
%             a = sizeView(2);
%             b = sizeView(1);
            a = 640;
            b = 480;
            
            mu = 118;
            mv = 118;
%             mu = a/(2*rMax);
%             mv = b/(2*rMax);
            u0 = a/2;
            v0 = b/2;
        end
        
        function [x, y] = getSensorPosition(C, pixelPosition, mu, mv, u0, v0)
            T = [(1/mu) 0 ; 0 (1/mv)];
            centeredPixelPositions = [(pixelPosition(2) - u0) ; (pixelPosition(3) - v0)];
            
            aux = T*centeredPixelPositions;
            
            x = aux(1);
            y = aux(2);
        end
        
        function realPosition = getRealPosition(C, m, z)
            lambda = 1;
            theta = C.parameters(1);
            psi = C.parameters(2);
            phi = C.parameters(3);
            t = [C.parameters(4); C.parameters(5); C.parameters(6)];
            focusLength = C.parameters(7);
            mu = C.parameters(8);
            mv = C.parameters(9);
            u0 = C.parameters(10);
            v0 = C.parameters(11);
            
            R = services.Math.getRotationMatrix(psi, theta, phi); 
            T = [R(1, 1) R(1, 2) t(1) ; R(2, 1) R(2, 2) t(2) ; R(3, 1) R(3, 2) t(3)];
            F = [focusLength 0 0 ; 0 focusLength 0 ; 0 0 1];
            M = [mu 0 u0 ; 0 mv v0 ; 0 0 1];
            I = M*F;

            error = 1;
            while (error > 0.0001)
                V = inv(T)*(inv(I)*[m(1) ; m(2) ; 1] - lambda*z*[R(1, 3) ; R(2, 3) ; R(3, 3)]);
                oldLambda = lambda;
                lambda = V(3);
                error = abs(lambda - oldLambda);
            end

            realPosition = [V(1)/V(3) V(2)/V(3) z];
        end
        
        function pixelPosition = getRealPixelPosition(C, X)
            theta = 0.0256;
            theta = C.parameters(1);
            psi = C.parameters(2);
            phi = C.parameters(3);
            t = [C.parameters(4); C.parameters(5); C.parameters(6)];
            focusLength = C.parameters(7);
            mu = C.parameters(8);
            mv = C.parameters(9);
            u0 = C.parameters(10);
            v0 = C.parameters(11);
            
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
        
        function [gradient, neutralErrorValue] = calculateGradient(C, X, angles, t, focusLength, M, pixelValuesCalibrationPoint, viewsLength)
            % Parameters
            delta = 0.0001;
            
            viewNumber = pixelValuesCalibrationPoint(1);
            theta = angles(viewNumber, 1);
            psi = angles(viewNumber, 2);
            phi = angles(viewNumber, 3); 
            t = [
               t(viewNumber, 1) ; 
               t(viewNumber, 2) ; 
               t(viewNumber, 3) ; 
            ];
            for i=1:(6*viewsLength)
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
            neutralErrorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            
            % psi
            theta = theta + delta;
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            theta = theta - delta;
            
            gradient(3*(viewNumber-1)+1) = (errorValue - neutralErrorValue)/delta;
            
            % theta
            psi = psi + delta;
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            psi = psi - delta;
            
            gradient(3*(viewNumber-1)+2) = (errorValue - neutralErrorValue)/delta;
            
            % phi
            phi = phi + delta;
            R = services.Math.getRotationMatrix(psi, theta, phi);
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            phi = phi - delta;
            
            gradient(3*(viewNumber-1)+3) = (errorValue - neutralErrorValue)/delta;
            
            % t1
            t(1) = t(1) + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            t(1) = t(1) - delta;
            
            gradient(3*viewsLength + 3*(viewNumber-1)+1) = (errorValue - neutralErrorValue)/delta;            
            % t2
            t(2) = t(2) + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            t(2) = t(2) - delta;
            
            gradient(3*viewsLength + 3*(viewNumber-1)+2) = (errorValue - neutralErrorValue)/delta;  
            
            % t3
            t(3) = t(3) + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            t(3) = t(3) - delta;
            
            gradient(3*viewsLength +3*(viewNumber-1)+3) = (errorValue - neutralErrorValue)/delta;  
            
            % focusLength
            focusLength = focusLength + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            focusLength = focusLength - delta;
            
            gradient(6*viewsLength + 1) = (errorValue - neutralErrorValue)/delta;
            
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
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            mu = mu - delta;
            
            gradient(6*viewsLength + 2) = (errorValue - neutralErrorValue)/delta; 
            
            % mv
            mv = mv + delta;
            pixelPosition = C.calculatePixelPosition(X, R, t, focusLength, [mu mv u0 v0]);
            errorValue = (pixelPosition(1) - pixelValuesCalibrationPoint(2))^2 + (pixelPosition(2) - pixelValuesCalibrationPoint(3))^2;
            mv = mv - delta;
            
            gradient(6*viewsLength + 3) = (errorValue - neutralErrorValue)/delta;   
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
    end
end
