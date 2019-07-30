classdef Math
    properties (Access = private)
    end
    
    methods (Static)
        function R = getRotationMatrix(psi, theta, phi)
            R(1,1) = cos(theta)*cos(phi); 
            R(2,1) = cos(theta)*sin(phi);
            R(3,1) = -sin(theta);
            R(1,2) = sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi); 
            R(2,2) = sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi); 
            R(3,2) = sin(psi)*cos(theta);    
            R(1,3) = cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi); 
            R(2,3) = cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi);
            R(3,3) = cos(psi)*cos(theta); 
        end
        
        function [r, phi] = getPolarCoordinates(euclideanCoordinates)
            phi = atan2(euclideanCoordinates(2), euclideanCoordinates(1));
            if (euclideanCoordinates(1) == 0) 
                r = abs(euclideanCoordinates(2));
            elseif (euclideanCoordinates(1) == 0 && euclideanCoordinates(2) == 0)
                r = 0;
            else
                r = euclideanCoordinates(1)/cos(phi);
            end
        end
        
%         function [x_img, y_img] = toPixelBasis(point, topographicMap, measurement)
%             [w_real, h_real] = measurement.getChamberSize();
%             [h_img, w_img] = size(topographicMap.map);
%             x_img = floor(w_img*(0.5 + (point(1)/w_real)));
%             y_img = floor(h_img*(0.5 + (point(2)/h_real)));
%             
%         end
        
        function robotCenter = getRobotCenter(sensorsPosition, theta, robot)
            sensorsRobotPosition = robot.sensorsPosition;
            neutralTheta = atan2(sensorsRobotPosition(2), sensorsRobotPosition(1));
            lengthSensorsVector = (sensorsRobotPosition(1)^2 + sensorsRobotPosition(2)^2)^0.5;
            
            robotCenter = [
                sensorsPosition(1) - lengthSensorsVector*cos(theta + neutralTheta) 
                sensorsPosition(2) - lengthSensorsVector*sin(theta + neutralTheta) 
            ];    
        end
    end
end
