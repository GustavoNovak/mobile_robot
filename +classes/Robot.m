classdef Robot
    properties
        connection;
        cameras;
        diameter;
        toolZone;
        sensorsPosition;
        maxLinearVelocity;
        maxAngularVelocity;
    end
    
    methods        
        function R = Robot(diameter, toolZone, sensorsPosition, maxLinearVelocity, maxAngularVelocity)
            R.connection = tcpclient('127.0.0.1', 444);
            R.cameras = classes.Cameras();
            R.diameter = diameter;
            R.toolZone = R.generateToolZone(toolZone);
            R.sensorsPosition = sensorsPosition;
            R.maxLinearVelocity = maxLinearVelocity;
            R.maxAngularVelocity = maxAngularVelocity;
        end
        
        function R = setVelocity(R, V, t)
            vx = strcat('VX', num2str(V(1)));
            vy = strcat('VY', num2str(V(2)));
            phi = strcat('PHI', num2str(V(3)));
            data = uint8(strcat(vx, vy, phi));
            write(R.connection, data);
        end
        
        function [x, y, phi] = getPosition(R)
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
