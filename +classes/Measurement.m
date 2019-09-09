classdef Measurement
    properties
        type;
        data;
        chamberSize;
    end
    
    methods
        function M = Measurement(type, data, chamberSize)
            M.type = type;
            M.data = data;
            M.chamberSize = chamberSize;
        end
        
        function points = getMeasurementPoints(M)
            size = M.chamberSize;
            
            width = size(1);
            height = size(2);
            count = 0;
            switch M.type
                case 'mesh'
                    x = M.data(1) / 2;
                    while x < width/2
                        y = M.data(2) / 2;
                        while y < height/2
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y + M.data(2);  
                        end
                        
                        y = - M.data(2) / 2;
                        while -y < height/2
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y - M.data(2);  
                        end
                        
                        x = x + M.data(1);
                    end
                    
                    x = - M.data(1) / 2;
                    while -x <  width/2
                        y = M.data(2) / 2;
                        while y < height/2
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y + M.data(2);  
                        end
                        
                        y = - M.data(2) / 2;
                        while -y < height/2
                            count = count + 1;
                            points(count, :) = [x y];
                            y = y - M.data(2);  
                        end
                        x = x - M.data(1);
                    end
            end
        end
        
        function display(M)
            disp('type: ');disp([M.type]);  
            disp('data: ');disp([M.data]);
            disp('ledsPosition: ');disp([M.ledsPosition]);
        end
        
        % Setters
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
        function M = set.chamberSize(M, value)
            if (services.Validator.isMatrix(value, [1 2], 'float'))
                M.chamberSize = value;
            else
                error('Invalid chamber size value');
            end
        end
    end
end
