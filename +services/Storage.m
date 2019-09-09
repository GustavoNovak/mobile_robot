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
        
        function inputCamerasCalibration = getInputCamerasCalibration()
            inputCamerasCalibration = textread('storage/input_cameras_calibration.dat');
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

