classdef RobotController
    properties
        robot;
        path;
    end
    
    methods
        function R = RobotController(robot, path)
            R.robot = robot;
            R.path = path;
        end
        
        function moveInPath(R)
            Kp = 1.5;
            Ki = 0.0;
            Kp_phi = 1.3;
            Ki_phi = 0.0;
            for i=1:length(R.path)
                if (R.path(i,4) == 0) 
                    % Control just in linear velocity
                    errorX = 0;
                    errorY = 0;
                    integralErrorX = 0;
                    integralErrorY = 0;
                    positionError = 100;
                    velocityMagnitude = 0;
                    tic;
                    x = -999999;
                    y = -999999;
                    phi = -999999;
                    count = 1;
                    while (positionError > 10 || velocityMagnitude > 50 || x == -999999)
                        [x_new, y_new, phi_new] = R.robot.getPosition();
                        if (x_new ~= -999999)
                            x = x_new;
                            y = y_new;
                            phi = phi_new;
                        end
                        if (x ~= -999999)
                            t = toc;
%                             oldErrorX = errorX;
%                             oldErrorY = errorY;
%                             errorX = R.path(i, 1) - x; 
%                             errorY = R.path(i, 2) - y; 
%                             integralErrorX = integralErrorX + ((oldErrorX + errorX)/2)*t;
%                             integralErrorY = integralErrorY + ((oldErrorY + errorY)/2)*t;

                            direction = [(R.path(i, 1) - x) (R.path(i, 2) - y)];
                            error = norm(direction);
                            direction = direction/error;
                            Vx_real = Kp*error*direction(1);
                            Vy_real = Kp*error*direction(2);
%                             Vx_real = Kp*errorX + Ki*integralErrorX;
%                             Vy_real = Kp*errorY + Ki*integralErrorY;
% 
                            Vx = cos(phi)*Vx_real + sin(phi)*Vy_real;
                            Vy = -sin(phi)*Vx_real + cos(phi)*Vy_real;
                            velocityMagnitude = norm([Vx Vy])

                            Vx = Vx/1000;
                            Vy = Vy/1000;

                            if (velocityMagnitude > R.robot.maxLinearVelocity)
                                Vx = Vx * (R.robot.maxLinearVelocity/velocityMagnitude);
                                Vy = Vy * (R.robot.maxLinearVelocity/velocityMagnitude);
                            end
                            positionError = error
%                             positionError = norm([errorX errorY])
                            R.robot = R.robot.setVelocity([Vx Vy 0], t);
                            tic;
                            count = count + 1;
                            if (count > 100)
                                break
                            end
                        end
                    end
                    disp('Position Achived');
                else
                    % Control in linear and angular velocity
                    errorX = 0;
                    errorY = 0;
                    errorPhi = 0;
                    integralErrorX = 0;
                    integralErrorY = 0;
                    integralErrorPhi = 0;
                    positionError = 0;
                    angularError = 0;
                    velocityMagnitude = 0;
                    Vphi = 0.0;
                    tic;
                    x = -999999;
                    y = -999999;
                    phi = -999999;
                    count = 1;
                    while (positionError > 2 || angularError > 0.02 || velocityMagnitude > 10 || Vphi > 0.1 || x == -999999)
                        [x_new, y_new, phi_new] = R.robot.getPosition();
                        if (x_new ~= -999999)
                            x = x_new;
                            y = y_new;
                            phi = phi_new;
                        end
                        if (x ~= -999999)
                            t = toc
%                             oldErrorX = errorX;
%                             oldErrorY = errorY;
                            
                            oldErrorPhi = errorPhi;
%                             errorX = R.path(i, 1) - x; 
%                             errorY = R.path(i, 2) - y;
                            if(R.path(i, 3) >= 0 && phi >= 0)
                                errorPhi = R.path(i, 3) - phi;
                            elseif(R.path(i, 3) < 0 && phi >= 0)
                                positiveDistance = 2*pi - phi + R.path(i, 3);
                                if(positiveDistance <= pi)
                                    errorPhi = positiveDistance;
                                else
                                    errorPhi = phi - R.path(i, 3);
                                end
                            elseif(R.path(i, 3) >= 0 && phi < 0)
                                positiveDistance = R.path(i, 3) - phi;
                                if(positiveDistance <= pi)
                                    errorPhi = positiveDistance;
                                else
                                    errorPhi = 2*pi + phi - R.path(i, 3);
                                end                            
                            elseif(R.path(i, 3) < 0 && phi < 0)
                                errorPhi = R.path(i, 3) - phi;
                            end
%                             integralErrorX = integralErrorX + ((oldErrorX + errorX)/2)*t;
%                             integralErrorY = integralErrorY + ((oldErrorY + errorY)/2)*t;
                            integralErrorPhi = integralErrorPhi + ((oldErrorPhi + errorPhi)/2)*t;
                            
                            direction = [(R.path(i, 1) - x) (R.path(i, 2) - y)];
                            error = norm(direction);
                            direction = direction/error;
                            Vx_real = Kp*error*direction(1);
                            Vy_real = Kp*error*direction(2);

%                             Vx_real = Kp*errorX + Ki*integralErrorX;
%                             Vy_real = Kp*errorY + Ki*integralErrorY;
     
                            Vphi = Kp_phi*errorPhi + Ki_phi*integralErrorPhi;
                            if (abs(Vphi) > R.robot.maxAngularVelocity)
                                Vphi = sign(Vphi)*R.robot.maxAngularVelocity;
                            end
                            Vx = cos(phi)*Vx_real + sin(phi)*Vy_real;
                            Vy = -sin(phi)*Vx_real + cos(phi)*Vy_real;
                            velocityMagnitude = norm([Vx Vy])
                            if (velocityMagnitude > R.robot.maxLinearVelocity)
                                Vx = Vx * (R.robot.maxLinearVelocity/velocityMagnitude);
                                Vy = Vy * (R.robot.maxLinearVelocity/velocityMagnitude);
                            end
                            positionError = error
                            angularError = abs(errorPhi)

                            Vx = Vx/1000;
                            Vy = Vy/1000;

                            R.robot = R.robot.setVelocity([Vx Vy Vphi], t);
                            tic;
                            count = count + 1;
                            if (count > 1000)
                                break
                            end
                        end
                    end
                    t = toc;
                    R.robot = R.robot.setVelocity([0 0 0], t);
                    disp('Measuring...');
                    pause(10);
                    disp('Measure realized');
                end
            end
        end
    end
end
