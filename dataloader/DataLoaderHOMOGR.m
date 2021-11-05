classdef DataLoaderHOMOGR < DataLoader
    %DataLoaderEPFL dataloader class for EPFL
    
    properties
        basePath = './datasets/homogr/';
%                 matcherParam = struct('FeatType', 'ORB', 'MatchThreshold', 30, ...
%                     'MaxRatio', 0.7, 'Unique', true, 'Approximate', false);
        matcherParam = struct('FeatType', 'SURF', 'MatchThreshold', 30, ...
            'MaxRatio', 0.7, 'Unique', true, 'Approximate', false);
    end
    
    methods
        function [scenes] = listScenes(obj)
            scenes = {};
            listing = dir(obj.basePath);
            numResults = size(listing, 1);
            
            for i = 1:numResults
                if listing(i).isdir == 1
                    continue;
                end
                filename = listing(i).name;
                if length(filename) <= 9
                    continue;
                end
                if ~strcmp(filename(end-8:end), '_vpts.mat')
                    continue;
                end
                
                scenes{end+1} = filename(1:end-9);
            end
        end
        
        function [frames] = listViewsByScene(obj, scene)            
            frames = [1, 2];
        end
        
        function [viewStruct] = loadViewPair(obj, scene, viewPair)
            assert(size(viewPair, 2) == 2);
            assert(all(viewPair(1, :) == [1, 2]));
            
            images = cell(1, 2);
            try
                images{1} = imread([obj.basePath, scene, 'A.png']);
                images{2} = imread([obj.basePath, scene, 'B.png']);
            catch
                images{1} = imread([obj.basePath, scene, 'A.jpg']);
                images{2} = imread([obj.basePath, scene, 'B.jpg']);
            end
            
            fd = load([obj.basePath, scene, '_vpts.mat']);
            valCorresp = cell(1, 2);
            valCorresp{1} = Ordered2dPoints(fd.validation.pts(1:3, :));
            valCorresp{2} = Ordered2dPoints(fd.validation.pts(4:6, :));
            
            viewStruct = struct();
            viewStruct.images = images;
            viewStruct.H = HomoMat(fd.validation.model).inverse();
            viewStruct.valCorresp = valCorresp;
        end
    end
end
