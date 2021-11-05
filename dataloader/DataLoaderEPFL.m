classdef DataLoaderEPFL < DataLoaderLoadByView
    %DataLoaderEPFL dataloader class for EPFL
    
    properties
        basePath = './datasets/EPFL/';
        %         matcherParam = struct('FeatType', 'ORB', 'MatchThreshold', 30, ...
        %             'MaxRatio', 0.7, 'Unique', true, 'Approximate', false);
        matcherParam = struct('FeatType', 'SURF', 'MatchThreshold', 30, ...
            'MaxRatio', 0.7, 'Unique', true, 'Approximate', false);
    end
    
    methods
        function [scenes] = listScenes(obj)
            scenes = DataLoader.listDirNames(obj.basePath);
        end
        
        function [frames] = listViewsByScene(obj, scene)            
            scenePath = [obj.basePath, scene];
            frames = [];
            listing = dir(scenePath);
            numResults = size(listing, 1);
            
            if exist([scenePath, '/0000.png'], 'file')
                for i = 1:numResults
                    filename = listing(i).name;
                    if length(filename) ~= 8
                        continue;
                    end
                    if ~strcmp(filename(5:8), '.png')
                        continue;
                    end
                    frames(end+1) = str2num(filename(1:4));
                end
            elseif exist([scenePath, '/rdimage.000.ppm'], 'file')
                for i = 1:numResults
                    filename = listing(i).name;
                    if length(filename) ~= 15
                        continue;
                    end
                    if ~strcmp(filename(12:15), '.ppm')
                        continue;
                    end
                    frames(end+1) = str2num(filename(9:11));
                end
            else
                error('wrong EPFL datapath')
            end
        end
        
        function [viewStruct] = loadView(obj, scene, view)
            dataPath = [obj.basePath, scene, '/'];
            try
                imgPath = sprintf('rdimage.%03d.ppm', view);
                [calibration, R, t, imgSize] = DataLoaderEPFL.readCalibrationOrientation_EPFL(dataPath, imgPath);
            catch
                imgPath = sprintf('%04d.png', view);
                [calibration, R, t, imgSize] = DataLoaderEPFL.readCalibrationOrientation_EPFL(dataPath, imgPath);
            end
            pose = [R, t];
            image = imread([dataPath, imgPath]);
            
            viewStruct = struct();
            viewStruct.image = image;
            viewStruct.pose = Pose(pose);
            viewStruct.calibration = calibration;
        end
    end
    
    methods(Static,Access=private)
        function [K,R,t,im_size]=readCalibrationOrientation_EPFL(image_path,image_name)
            %READCALIBRATIONORIENTATION_EPFL Gets the true calibration and orientation
            % of a camera in an EPFL dataset.

            filename=strcat(image_path,image_name,'.camera');
            calibration_file = fopen(filename,'r');

            K=[str2num(fgetl(calibration_file));...
                str2num(fgetl(calibration_file));...
                str2num(fgetl(calibration_file))];

            fgetl(calibration_file);

            R=[str2num(fgetl(calibration_file));...
                str2num(fgetl(calibration_file));...
                str2num(fgetl(calibration_file))].';

            t=-R*[str2num(fgetl(calibration_file))].';

            im_size=[str2num(fgetl(calibration_file))].';

            fclose(calibration_file);

        end
    end
end
