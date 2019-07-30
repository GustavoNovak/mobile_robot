classdef Storage
    properties (Access = private)
    end
    
    methods (Static)
        function topographicMap = getTopographicMap()
            topographicMap = imread('storage/topographic_map.png');
        end
    end
end

