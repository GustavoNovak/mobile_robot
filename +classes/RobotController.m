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
        
        function moveInPath(R, camerasClass, cameras)
            Kp = 1.5;
            Ki = 0.0;
            Kp_phi = 1.3;
            Ki_phi = 0.0;
            for i=1:length(R.path)
                R.path(i,:)
                if (R.path(i,4) == 0) 
                    % Control just in linear velocity
                    errorX = 0;
                    errorY = 0;
                    errorPhi = 0;
                    integralErrorX = 0;
                    integralErrorY = 0;
                    integralErrorPhi = 0;
                    positionError = 100;
                    angularError = 0;
                    velocityMagnitude = 0;
                    Vphi = 0.0;
                    tic;
                    x = -999999;
                    y = -999999;
                    phi = -999999;
                    count = 1;
                    while (positionError > 10 || angularError > 0.05 || velocityMagnitude > 50 || x == -999999)
                        if(x == -999999)
                            robotPosition = [0 0 0];
                        else
                            robotPosition = [x y phi];
                        end
                        
                        [x_new, y_new, phi_new] = R.robot.getPosition(robotPosition, camerasClass, cameras);
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
                            errorPhi = R.path(i, 3) - phi;
%                             integralErrorX = integralErrorX + ((oldErrorX + errorX)/2)*t;
%                             integralErrorY = integralErrorY + ((oldErrorY + errorY)/2)*t;
%                             integralErrorPhi = integralErrorPhi + ((oldErrorPhi + errorPhi)/2)*t;
                            
                            direction = [(R.path(i, 1) - x) (R.path(i, 2) - y) errorPhi];
                            error = norm(direction);
                            direction = direction/error;
                            Vx_real = Kp*error*direction(1);
                            Vy_real = Kp*error*direction(2);
                            Vx = cos(phi)*Vx_real + sin(phi)*Vy_real;
                            Vy = -sin(phi)*Vx_real + cos(phi)*Vy_real;
                            velocityMagnitude = norm([Vx Vy]);

%                             Vx_real = Kp*errorX + Ki*integralErrorX;
%                             Vy_real = Kp*errorY + Ki*integralErrorY;
     
                            Vphi = Kp_phi*error*direction(3);
                            if (velocityMagnitude > R.robot.maxLinearVelocity)
                                Vx = Vx * (R.robot.maxLinearVelocity/velocityMagnitude);
                                Vy = Vy * (R.robot.maxLinearVelocity/velocityMagnitude);
                                Vphi = Vphi * (R.robot.maxLinearVelocity/velocityMagnitude);
                            end
                            if (abs(Vphi) > R.robot.maxAngularVelocity)
                                Vx = Vx * (R.robot.maxAngularVelocity/abs(Vphi));
                                Vy = Vy * (R.robot.maxAngularVelocity/abs(Vphi));
                                Vphi = Vphi * (R.robot.maxAngularVelocity/abs(Vphi));
                            end
                            positionError = norm([(R.path(i, 1) - x) (R.path(i, 2) - y)]);
                            angularError = errorPhi;

                            Vx = Vx/1000;
                            Vy = Vy/1000;

                            R.robot = R.robot.setVelocity([Vx Vy Vphi]);
                            tic;
                            count = count + 1;
                            if (count > 1000)
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
                    while (positionError > 5 || angularError > 0.03 || velocityMagnitude > 10 || Vphi > 0.1 || x == -999999)
                        if(x == -999999)
                            robotPosition = [0 0 0];
                        else
                            robotPosition = [x y phi];
                        end
                        
                        tic;
                        [x_new, y_new, phi_new] = R.robot.getPosition(robotPosition, camerasClass, cameras);
                        t = toc;
                        if (x_new ~= -999999)
                            x = x_new;
                            y = y_new;
                            phi = phi_new;
                        end
                        if (x ~= -999999)
                            t = toc;
%                             oldErrorX = errorX;
%                             oldErrorY = errorY;
                            
                            oldErrorPhi = errorPhi;
%                             errorX = R.path(i, 1) - x; 
%                             errorY = R.path(i, 2) - y;
                            errorPhi = R.path(i, 3) - phi;
%                             integralErrorX = integralErrorX + ((oldErrorX + errorX)/2)*t;
%                             integralErrorY = integralErrorY + ((oldErrorY + errorY)/2)*t;
%                             integralErrorPhi = integralErrorPhi + ((oldErrorPhi + errorPhi)/2)*t;
                            
                            direction = [(R.path(i, 1) - x) (R.path(i, 2) - y) errorPhi];
                            error = norm(direction);
                            direction = direction/error;
                            Vx_real = Kp*error*direction(1);
                            Vy_real = Kp*error*direction(2);
                            Vx = cos(phi)*Vx_real + sin(phi)*Vy_real;
                            Vy = -sin(phi)*Vx_real + cos(phi)*Vy_real;
                            velocityMagnitude = norm([Vx Vy]);

%                             Vx_real = Kp*errorX + Ki*integralErrorX;
%                             Vy_real = Kp*errorY + Ki*integralErrorY;
     
                            Vphi = Kp_phi*error*direction(3);
                            if (velocityMagnitude > R.robot.maxLinearVelocity)
                                Vx = Vx * (R.robot.maxLinearVelocity/velocityMagnitude);
                                Vy = Vy * (R.robot.maxLinearVelocity/velocityMagnitude);
                                Vphi = Vphi * (R.robot.maxLinearVelocity/velocityMagnitude);
                            end
                            if (abs(Vphi) > R.robot.maxAngularVelocity)
                                Vx = Vx * (R.robot.maxAngularVelocity/abs(Vphi));
                                Vy = Vy * (R.robot.maxAngularVelocity/abs(Vphi));
                                Vphi = Vphi * (R.robot.maxAngularVelocity/abs(Vphi));
                            end
                            positionError = norm([(R.path(i, 1) - x) (R.path(i, 2) - y)]);
                            angularError = errorPhi;

                            Vx = Vx/1000;
                            Vy = Vy/1000;

                            R.robot = R.robot.setVelocity([Vx Vy Vphi]);
                            tic;
                            count = count + 1;
                            if (count > 1000)
                                break
                            end
                        end
                    end
                    t = toc;
                    R.robot = R.robot.setVelocity([0 0 0]);
                    disp('Measuring...');
                    pause(10);
                    disp('Measure realized');
                end
            end
        end
    end
end
