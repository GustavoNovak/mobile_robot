classdef Measurement
    properties
        measurementTime = 60;
        type;
        data;
        ledsPosition;
    end
    
    methods
        function M = Measurement(type, data, ledsPosition)
            M.type = type;
            M.data = data;
            M.ledsPosition = ledsPosition;
        end
        
        function [width, height] = getChamberSize(M)
            firstLed = M.ledsPosition(1,:);
            secondLed = M.ledsPosition(2,:);
            thirdLed = M.ledsPosition(3,:);
            if (firstLed(1) == secondLed(1))
                height = abs(firstLed(2) - secondLed(2));
                width = abs(secondLed(1) - thirdLed(3));
            else
                width = abs(firstLed(1) - secondLed(1));
                height = abs(secondLed(2) - thirdLed(2));
            end
        end
        
        function points = getMeasurementPoints(M)
            [width, height] = M.getChamberSize();
            
            width = width - 10;
            height = height - 10;
            count = 0;
            switch M.type
                case 'mesh'
                    x = M.data(1) / 2;
                    while x < (width/2 - M.data(1))
                        y = M.data(2) / 2;
                        while y < (height/2 - M.data(2))
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y + M.data(2);  
                        end
                        
                        y = - M.data(2) / 2;
                        while -y < (height/2 - M.data(2))
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y - M.data(2);  
                        end
                        
                        x = x + M.data(1);
                    end
                    
                    x = - M.data(1) / 2;
                    while -x <  (width/2 - M.data(1))
                        y = M.data(2) / 2;
                        while y < (height/2 - M.data(2))
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y + M.data(2);  
                        end
                        
                        y = - M.data(2) / 2;
                        while -y < (height/2 - M.data(2))
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y - M.data(2);  
                        end
                        x = x - M.data(1);
                    end
            end
        end
        
        function display(M)
            disp('measurementTime: ');disp([M.measurementTime]);
            disp('type: ');disp([M.type]);  
            disp('data: ');disp([M.data]);
            disp('ledsPosition: ');disp([M.ledsPosition]);
        end
        
        % Setters
        function M = set.measurementTime(M, value)
            if (services.Validator.isGreaterThan(value, 0, 'integer'));
                M.measurementTime = value;
            else
                error('Invalid measurement time')
            end
        end
        function M = set.type(M, value)
            possTypes = {'mesh'};
            if (services.Validator.inList(value, possTypes));
                M.type = value;
            else
                error('Invalid type')
            end
        end
        function M = set.data(M, value)
            switch M.type
                case 'mesh'
                    if (services.Validator.isMatrix(value, [1 2], 'float'))
                        M.data = value;
                    else
                        error('Invalid data, you need to choose an numeric matrix with size equals to [1 2]');
                    end  
                otherwise
                    error('You need to define before the type of Measurement');
            end
        end
        function M = set.ledsPosition(M, value)
            if (services.Validator.isMatrix(value, [4 2], 'float'))
                M.ledsPosition = value;
            else
                error('Invalid led positions values');
            end
        end
    end
end
