classdef Storage
    properties (Access = private)
    end
    
    methods (Static)
        function topographicMap = getTopographicMap()
            topographicMap = imread('storage/topographic_map.png');
        end
        
        function realTopographicMap = getRealTopographicMap()
            realTopographicMap = imread('storage/real_topographic_map.png');
        end
        
        function cameraParameters = getCameraParameters()
            cameraParameters = textread('storage/camera_calibration.dat');
        end
    end
end

