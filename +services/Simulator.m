classdef Simulator
    properties (Access = private)
    end
    
    methods (Static)
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
            t(1,1) = 0;
            t(2,1) = 200;
            t(3,1) = 2000;
            
            count = 0;
            for i = 1:length(theta)
                R = services.Math.getRotationMatrix(psi(i), theta(i), phi(i));
                T = [
                    R(1,1) R(1,2) R(1,3) t(1,i) ; 
                    R(2,1) R(2,2) R(2,3) t(2,i) ; 
                    R(3,1) R(3,2) R(3,3) t(3,i) ; 
                    0 0 0 1
                ];
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
            [M(1, 1) M(1, 2) pixelCenter(1) ; M(2, 1) M(2, 2) pixelCenter(2) ; 0 0 1]*[focusLength 0 0  ; 0 focusLength 0  ; 0 0 1 ]*[R(1,1) R(1,2) t(1,1) ; R(2,1) R(2,2) t(2,1) ; R(3,1) R(3,2) t(3,1)]
        end
    end
end

