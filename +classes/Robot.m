classdef Robot
    properties
        connection = '';
        diameter;
        toolZone;
        sensorsPosition;
        maxLinearVelocity;
        maxAngularVelocity;
    end
    
    methods        
        function R = Robot(diameter, toolZone, sensorsPosition, maxLinearVelocity, maxAngularVelocity)
            R.connection = tcpclient('127.0.0.1', 444);
            R.diameter = diameter;
            R.toolZone = R.generateToolZone(toolZone);
            R.sensorsPosition = sensorsPosition;
            R.maxLinearVelocity = maxLinearVelocity;
            R.maxAngularVelocity = maxAngularVelocity;
        end
        
        function R = setVelocity(R, V)
            vx = strcat('VX', num2str(V(1)));
            vy = strcat('VY', num2str(V(2)));
            phi = strcat('PHI', num2str(V(3)));
            data = uint8(strcat(vx, vy, phi));
            write(R.connection, data);
        end
        
        function [x, y, phi] = getPosition(R, robotPosition, camerasClass, cameras, velocity, timer, countSettedVelocity)
            [x, y, phi] = camerasClass.getRobotPosition(robotPosition, cameras, velocity, timer, countSettedVelocity, R);
            x = x + 50*cos(phi);
            y = y + 50*sin(phi);
        end
        
        function mesh = generateToolZone(R, toolZone)
            lengthToolZone = size(toolZone);
            lengthToolZone = lengthToolZone(1);
            count = 0;
            mesh = [];
            if(lengthToolZone > 0)
                for i=1:lengthToolZone
                    heightMax = toolZone(i, 1);
                    widthMax = toolZone(i, 2);
                    angle = toolZone(i, 3); 
                    directionVector = [sin(angle) -cos(angle)];
                    height = ((R.diameter/2)^2 - (widthMax/2)^2)^0.5;
                    while (height <= heightMax)
                        t = -widthMax/2;
                        while t <= (widthMax/2)
                            P0 = height*[cos(angle) sin(angle)];
                            newNode = P0 + t*directionVector;
                            if(norm(newNode) > (R.diameter/2))
                                [r, phi] = services.Math.getPolarCoordinates(newNode);
                                count = count + 1;
                                mesh(count, :) = [phi r];
                            end  
                            t = t + 20;
                        end
                        
                        if(t > (widthMax/2))
                            t = widthMax/2;
                            P0 = height*[cos(angle) sin(angle)];
                            newNode = P0 + t*directionVector;
                            if(norm(newNode) > (R.diameter/2))
                                [r, phi] = services.Math.getPolarCoordinates(newNode);
                                count = count + 1;
                                mesh(count, :) = [phi r];
                            end                        
                        end
                        height = height + 30;
                    end
                    
                    if(height > heightMax)
                        height = heightMax;
                        t = -widthMax/2;
                        while t <= (widthMax/2)
                            P0 = height*[cos(angle) sin(angle)];
                            newNode = P0 + t*directionVector;
                            if(norm(newNode) > (R.diameter/2))
                                [r, phi] = services.Math.getPolarCoordinates(newNode);
                                count = count + 1;
                                mesh(count, :) = [phi r];
                            end
                            t = t + 20;
                        end
                        
                        if(t > (widthMax/2))
                            t = widthMax/2;
                            P0 = height*[cos(angle) sin(angle)];
                            newNode = P0 + t*directionVector;
                            if(norm(newNode) > (R.diameter/2))
                                [r, phi] = services.Math.getPolarCoordinates(newNode);
                                count = count + 1;
                                mesh(count, :) = [phi r];
                            end                       
                        end                       
                    end
                end
            end
%             mesh
%             for i=1:length(mesh)
%                 plot(mesh(i, 1), mesh(i, 2), 'o');
%             end
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
