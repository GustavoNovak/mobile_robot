classdef Validator
    properties (Access = private)
    end
    
    methods (Static)
        function response = isClass(value, class)
            if (isa(value, class))
                response = true;
            else
                response = false;
            end
        end
        
        function response = isImage(value, type)
            if (all( value(:)==0 | value(:)==1)) 
                response = true;
            else
                response = false;
            end
        end
        
        function response = inList(value, list)
            switch value
                case list
                    response = true;
                otherwise
                    response = false;
            end
        end
        
        function response = isMatrix(value, sizeRef, type)
            sizeMatrix = size(value);
            switch type
                case 'float'
                    if (sizeMatrix(1) == sizeRef(1) && sizeMatrix(2) == sizeRef(2))
                        response = true;
                        for i = 1:sizeMatrix(1)
                            for j = 1:sizeMatrix(2)
                                if (~isnumeric(value(i,j)))
                                    response = false;
                                end
                            end
                        end
                    else
                        response = false;
                    end
            end
        end
        
        function response = isGreaterThan(value, reference, type)
            if (isnumeric(value))
                switch type
                    case 'integer'
                        if (value > reference && floor(value)==ceil(value))
                            response = true;
                        else
                            response = false;
                        end
                    case 'float'
                        if (value > reference)
                            response = true;
                        else
                            response = false;
                        end
                    otherwise
                        error('Validation type doesnt exist')
                end
            else
                response = false;
            end
        end
        function response = isLowerThan(value, reference, type)
            switch type
                case 'integer'
                    if (value < reference && floor(value)==ceil(value))
                        response = true;
                    else
                        response = false;
                    end
                case 'float'
                    if (isnumeric(value) && value < reference)
                        response = true;
                    else
                        response = false;
                    end
                otherwise
                    error('Validation type doesnt exist')
            end
        end
        function response = isGreaterEqualsTo(value, reference, type)
            switch type
                case 'integer'
                    if (value >= reference && floor(value)==ceil(value))
                        response = true;
                    else
                        response = false;
                    end
                case 'float'
                    if (isnumeric(value) && value >= reference)
                        response = true;
                    else
                        response = false;
                    end
                otherwise
                    error('Validation type doesnt exist')
            end
        end
        function response = isLowerEqualsTo(value, reference, type)
            switch type
                case 'integer'
                    if (value <= reference && floor(value) == ceil(value))
                        response = true;
                    else
                        response = false;
                    end
                case 'float'
                    if (isnumeric(value) && value <= reference)
                        response = true;
                    else
                        response = false;
                    end
                otherwise
                    error('Validation type doesnt exist')
            end
        end
    end
end
