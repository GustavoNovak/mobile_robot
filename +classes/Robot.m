classdef Robot
    properties
        position = [0 0];
        connection;
        cameras;
        diameter;
        sensorsPosition;
        maxLinearVelocity;
        maxAngularVelocity;
        velocity = [0 0 0];
        pos = [200 74 -0.2103];
    end
    
    methods        
        function R = Robot(diameter, sensorsPosition, maxLinearVelocity, maxAngularVelocity)
            R.connection = tcpclient('127.0.0.1', 444);
            R.cameras = classes.Cameras();
            R.diameter = diameter;
            R.sensorsPosition = sensorsPosition;
            R.maxLinearVelocity = maxLinearVelocity;
            R.maxAngularVelocity = maxAngularVelocity;
        end
        
        function R = setVelocity(R, V, t)
            vx = strcat('VX', num2str(V(1)));
            vy = strcat('VY', num2str(V(2)));
            phi = strcat('PHI', num2str(V(3)));
%             disp(vx);
%             disp(vy);
%             disp(phi);
%             R.pos(1) = R.pos(1) + R.velocity(1)*t; 
%             R.pos(2) = R.pos(2) + R.velocity(2)*t; 
%             R.pos(3) = R.pos(3) + R.velocity(3)*t; 
%             R.velocity = [V(1) V(2) V(3)];
%             
            data = uint8(strcat(vx, vy, phi));
            write(R.connection, data);
        end
        
        function [x, y, phi] = getPosition(R)
%             errorLinear = 5;
%             errorAngular = 0.1;
%             x = R.pos(1) + random('Normal',0,1)*errorLinear
%             y = R.pos(2) + random('Normal',0,1)*errorLinear
%             phi = R.pos(3) + random('Normal',0,1)*errorAngular
%             pause(0.3);
            [x, y, phi] = R.cameras.getRobotPosition();
            x = x + 55*cos(phi);
            y = y + 55*sin(phi);
            pixelPosition = R.cameras.getRealPixelPosition([x ; y ; 0]);
            if (x > -9000 && y > -9000)
                plot(pixelPosition(1), pixelPosition(2), 'o'); 
            else
                plot(0, 0, 'o'); 
            end 
        end
        
        function delete(R)
            R.cameras.delete();
        end
        
        function display(R)
            disp('diameter: ');disp([R.diameter]);
            disp('sensorsPosition: ');disp([R.sensorsPosition]);  
            disp('maxLinearVelocity: ');disp([R.maxLinearVelocity]);
            disp('maxAngularVelocity: ');disp([R.maxAngularVelocity]);
        end
        
        % Setters
        function R = set.diameter(R, value)
            if (services.Validator.isGreaterEqualsTo(value, 0, 'float'))
                R.diameter = value;
            else
                error('Invalid diameter');
            end
        end
        function R = set.sensorsPosition(R, value)
            if (services.Validator.isMatrix(value, [1 2], 'float'))
                R.sensorsPosition = value;
            else
                error('Invalid sensor position values');
            end
        end
        function R = set.maxLinearVelocity(R, value)
            if (services.Validator.isGreaterThan(value, 0, 'float'))
                R.maxLinearVelocity = value;
            else
                error('Invalid max linear velocity');
            end
        end
        function R = set.maxAngularVelocity(R, value)
            if (services.Validator.isGreaterThan(value, 0, 'float'))
                R.maxAngularVelocity = value;
            else
                error('Invalid max angular velocity');
            end
        end
    end
end