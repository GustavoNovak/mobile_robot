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
            parameters(1, :) = textread('storage/camera_calibration_1.dat');
            parameters(2, :) = textread('storage/camera_calibration_2.dat');
            cameraParameters = parameters;
        end
        
        function inputCamerasCalibration = getInputCamerasCalibration()
            inputCamerasCalibration = textread('storage/input_cameras_calibration_1.dat');
        end
        
        function inputTopographicMapGeneration = getInputTopographicMapGeneration()
            inputTopographicMapGeneration = textread('storage/input_topographic_map_generation.dat');
        end
        
        function storePath(path)
            save('storage/path.mat','path');
        end
        
        function path = getPath()
            stored = matfile('storage/path.mat');
            path = stored.path;
        end
    end
end

