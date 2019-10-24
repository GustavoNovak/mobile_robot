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
            Kp_phi = 1.3;
            
            timer = tic;
            [x, y, phi] = R.robot.getPosition([0 0], camerasClass, cameras, [0 0 0], timer, 0);
            
            if(x < -10000)
                msgbox('Move robot to a position were cameras can mesure his position!');
                return;
            end
            
            pathSize = size(R.path);
            pathSize = pathSize(1);
            
            for i=1:pathSize
                R.path(i,:)
                if (R.path(i,4) == 0) 
                    % Control just in linear velocity
                    positionError = 100;
                    angularError = 0;
                    velocityMagnitude = 0;
                    Vx_real = 0;
                    Vy_real = 0;
                    Vx = 0;
                    Vy = 0;
                    Vphi = 0;
                    timer = tic;
                    
                    count = 1;
                    while (positionError > 10 || angularError > 0.05 || velocityMagnitude > 50)
                        [x_new, y_new, phi_new] = R.robot.getPosition([x y phi], camerasClass, cameras, [Vx Vy Vphi], timer, 0);
                        deltaT = toc(timer);
                        if (x_new > -10000)
                            x = x + deltaT*Vx_real + 0.2*(x_new -(x + deltaT*Vx_real));
                            y = y + deltaT*Vy_real + 0.2*(y_new -(y + deltaT*Vy_real));
                            phi = phi + deltaT*Vphi + 0.2*(phi_new -(phi + deltaT*Vphi));
                        else
                            disp('just Velocity');
                            x = x + deltaT*Vx_real;
                            y = y + deltaT*Vy_real;
                            phi = phi + deltaT*Vphi;
                        end 
                        
                        if (x > -10000 && y > -10000)
                            plot(x, y, 'o');
                            hold on;
                        else
                            plot(0, 0, 'o'); 
                            hold on;
                        end
                        
                        positionError = norm([(R.path(i, 1) - x) (R.path(i, 2) - y)]);
                        angularError = R.path(i, 3) - phi;

                        direction = [(R.path(i, 1) - x) (R.path(i, 2) - y) (R.path(i, 3) - phi)];

                        error = norm(direction);
                        direction = direction/error;

                        Vx_real = Kp*error*direction(1);
                        Vy_real = Kp*error*direction(2);

                        Vx = cos(phi)*Vx_real + sin(phi)*Vy_real;
                        Vy = -sin(phi)*Vx_real + cos(phi)*Vy_real;

                        velocityMagnitude = norm([Vx Vy]);

                        Vphi = Kp_phi*error*direction(3);
                        if (velocityMagnitude > R.robot.maxLinearVelocity)
                            Vx_real = Vx_real * (R.robot.maxLinearVelocity/velocityMagnitude);
                            Vy_real = Vy_real * (R.robot.maxLinearVelocity/velocityMagnitude);
                            
                            Vx = Vx * (R.robot.maxLinearVelocity/velocityMagnitude);
                            Vy = Vy * (R.robot.maxLinearVelocity/velocityMagnitude);
                            Vphi = Vphi * (R.robot.maxLinearVelocity/velocityMagnitude);
                        end
                        if (abs(Vphi) > R.robot.maxAngularVelocity)
                            Vx_real = Vx_real * (R.robot.maxAngularVelocity/abs(Vphi));
                            Vy_real = Vy_real * (R.robot.maxAngularVelocity/abs(Vphi));
                            
                            Vx = Vx * (R.robot.maxAngularVelocity/abs(Vphi));
                            Vy = Vy * (R.robot.maxAngularVelocity/abs(Vphi));
                            Vphi = Vphi * (R.robot.maxAngularVelocity/abs(Vphi));
                        end

                        Vx = Vx/1000;
                        Vy = Vy/1000;

                        R.robot.setVelocity([Vx Vy Vphi]);
                        timer = tic;
                        
                        count = count + 1;
                        if (count > 1000)
                            break
                        end
                    end
                    
                    R.robot.setVelocity([0 0 0]);
                    disp('Position Achived');
                else
                    % Control in linear and angular velocity
                    positionError = 100;
                    angularError = 0;
                    velocityMagnitude = 0;
                    Vx_real = 0;
                    Vy_real = 0;
                    Vx = 0;
                    Vy = 0;
                    Vphi = 0;
                    timer = tic;
                    
                    count = 1;
                    while (positionError > 10 || angularError > 0.05 || velocityMagnitude > 50 || Vphi > 0.1)
                        [x_new, y_new, phi_new] = R.robot.getPosition([x y phi], camerasClass, cameras, [Vx Vy Vphi], timer, 0);
                        deltaT = toc(timer);
                        if (x_new > -10000)                            
                            x = x + deltaT*Vx_real + 0.5*(x_new -(x + deltaT*Vx_real));
                            y = y + deltaT*Vy_real + 0.5*(y_new -(y + deltaT*Vy_real));
                            phi = phi + deltaT*Vphi + 0.5*(phi_new -(phi + deltaT*Vphi));
                        else
                            disp('just Velocity');
                            x = x + deltaT*Vx_real;
                            y = y + deltaT*Vy_real;
                            phi = phi + deltaT*Vphi;
                        end
                        
                        if (x > -10000 || y > -10000)
                            plot(x, y, 'o');
                            hold on;
                        else
                            plot(0, 0, 'o');
                            hold on;
                        end

                        positionError = norm([(R.path(i, 1) - x) (R.path(i, 2) - y)]);
                        angularError = R.path(i, 3) - phi;

                        direction = [(R.path(i, 1) - x) (R.path(i, 2) - y) (R.path(i, 3) - phi)];
                        error = norm(direction);
                        direction = direction/error;
                        Vx_real = Kp*error*direction(1);
                        Vy_real = Kp*error*direction(2);
                        Vx = cos(phi)*Vx_real + sin(phi)*Vy_real;
                        Vy = -sin(phi)*Vx_real + cos(phi)*Vy_real;
                        velocityMagnitude = norm([Vx Vy]);

                        Vphi = Kp_phi*error*direction(3);
                        if (velocityMagnitude > R.robot.maxLinearVelocity)
                            Vx_real = Vx_real * (R.robot.maxLinearVelocity/velocityMagnitude);
                            Vy_real = Vy_real * (R.robot.maxLinearVelocity/velocityMagnitude);
                            
                            Vx = Vx * (R.robot.maxLinearVelocity/velocityMagnitude);
                            Vy = Vy * (R.robot.maxLinearVelocity/velocityMagnitude);
                            Vphi = Vphi * (R.robot.maxLinearVelocity/velocityMagnitude);
                        end
                        if (abs(Vphi) > R.robot.maxAngularVelocity)
                            Vx_real = Vx_real * (R.robot.maxAngularVelocity/abs(Vphi));
                            Vy_real = Vy_real * (R.robot.maxAngularVelocity/abs(Vphi));
                            
                            Vx = Vx * (R.robot.maxAngularVelocity/abs(Vphi));
                            Vy = Vy * (R.robot.maxAngularVelocity/abs(Vphi));
                            Vphi = Vphi * (R.robot.maxAngularVelocity/abs(Vphi));
                        end

                        Vx = Vx/1000;
                        Vy = Vy/1000;

                        R.robot.setVelocity([Vx Vy Vphi]);
                        timer = tic;
                        
                        count = count + 1;
                        if (count > 1000)
                            break
                        end
                    end
                    
                    R.robot.setVelocity([0 0 0]);
                    disp('Measuring...');
                    pause(1);
                    disp('Measure realized');
                end
            end
        end
    end
end
