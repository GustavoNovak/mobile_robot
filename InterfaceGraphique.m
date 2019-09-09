classdef InterfaceGraphique
    properties (Access = private)
    end
    
    methods (Static)
        function InterfaceGraphique = InterfaceGraphique() 
        end
        
        function calibrateCameras(cameras)
            cameras.calibrate(3.7, pi/5);
        end
        
        function generateTopographicMap(cameras)
            img = getsnapshot(cameras.cameras(2));
            services.Images.generateTopographicMap(cameras, img);
        end
    end
    
end

