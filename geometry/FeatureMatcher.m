classdef FeatureMatcher
    %FeatureMatcher
    %   TODO: 1. save keypoints, features, matched features so that it can
    %   be reused later 2. load them
    
    properties
        matcherParam
    end
    
    methods
        function obj = FeatureMatcher(matcherParam)
            obj.matcherParam = matcherParam;
        end
        
        function [keypoints] = detectFeatures(obj, image)
            switch obj.matcherParam.FeatType
                case 'ORB'
                    keypoints = detectORBFeatures(image);
                case 'SURF'
                    keypoints = detectSURFFeatures(image, 'MetricThreshold', 500);
                case 'SIFT'
                    error('not implemented')
                case 'rootSIFT'
                    error('not implemented')
                otherwise
                    error('not implemented');
            end
        end
        
        function [features, valid_points] = extractFeatures(obj, image, keypoints)
            switch obj.matcherParam.FeatType
                case 'ORB'
                    [features, valid_points] = extractFeatures(image, keypoints, 'Method', 'ORB');
                case 'SURF'
                    [features, valid_points] = extractFeatures(image, keypoints, 'Method', 'SURF', 'Upright', true);
                case 'SIFT'
                    error('not implemented')
                case 'rootSIFT'
                    error('not implemented')
                otherwise
                    error('not implemented');
            end
        end
        
        function [idPairs, metrics] = matchFeatures(obj, features1, features2)
            [idPairs, metrics] = matchFeatures(features1, features2, ...
                'MatchThreshold', obj.matcherParam.MatchThreshold, ...
                'MaxRatio', obj.matcherParam.MaxRatio, ...
                'Unique', obj.matcherParam.Unique);
        end
        
        function [matchedPoints, metrics] = match2views(obj, images)
            assert(length(images) == 2);
            if size(images{1}, 3) == 3
                image1 = rgb2gray(images{1});
                image2 = rgb2gray(images{2});
            else
                image1 = images{1};
                image2 = images{2};
            end
            
            % keypoint detection
            keypoints1 = obj.detectFeatures(image1);
            keypoints2 = obj.detectFeatures(image2);
            
            % feature extraction
            [features1, valid_points1] = obj.extractFeatures(image1, keypoints1);
            [features2, valid_points2] = obj.extractFeatures(image2, keypoints2);
            
            % feature matching
            % FIXME: transpose
            [idPairs, metrics] = obj.matchFeatures(features1, features2);
            metrics = metrics';
            
            % get coordinates of coorespondences
            matchedPoints1 = valid_points1.Location(idPairs(:, 1), :)';
            matchedPoints2 = valid_points2.Location(idPairs(:, 2), :)';
            
            matchedPoints = cell(1, 2);
            matchedPoints{1} = Ordered2dPoints(matchedPoints1);
            matchedPoints{2} = Ordered2dPoints(matchedPoints2);
        end
        
        function [matchedPoints, metrics] = match3views(obj, images)
            assert(length(images) == 3);
            if size(images{1}, 3) == 3
                image1 = rgb2gray(images{1});
                image2 = rgb2gray(images{2});
                image3 = rgb2gray(images{3});
            else
                image1 = images{1};
                image2 = images{2};
                image3 = images{3};
            end
            
            
            % keypoint detection
            keypoints1 = obj.detectFeatures(image1);
            keypoints2 = obj.detectFeatures(image2);
            keypoints3 = obj.detectFeatures(image3);
            
            % feature extraction
            [features1, valid_points1] = obj.extractFeatures(image1, keypoints1);
            [features2, valid_points2] = obj.extractFeatures(image2, keypoints2);
            [features3, valid_points3] = obj.extractFeatures(image3, keypoints3);
            
            % feature matching
            [idPairs12, metrics12] = obj.matchFeatures(features1, features2);
            [idPairs13, metrics13] = obj.matchFeatures(features1, features3);
            
            % merge matched correspondences
            idPairs = [];
            metrics = [];
            for j = 1:size(idPairs12, 1)
                id1 = idPairs12(j, 1);
                j13Array = find(idPairs13(:, 1) == id1);
                if isempty(j13Array)
                    continue;
                end
                for j13 = j13Array
                    idPairs = [idPairs; idPairs12(j, 1), idPairs12(j, 2), idPairs13(j13, 2)];
                    metrics = [metrics; metrics12(j), metrics13(j13)];
                end
            end
            
            % it is possible that there is no mathced features among 3
            % views
            if isempty(idPairs)
                disp('failed to match any features across 3 views');
                matchedPoints1 = [];
                matchedPoints2 = [];
                matchedPoints3 = [];
                metrics = [];
                return;
            end
            
            % get coordinates of coorespondences
            matchedPoints1 = valid_points1.Location(idPairs(:, 1), :)';
            matchedPoints2 = valid_points2.Location(idPairs(:, 2), :)';
            matchedPoints3 = valid_points3.Location(idPairs(:, 3), :)';
                        
            matchedPoints = cell(1, 3);
            matchedPoints{1} = Ordered2dPoints(matchedPoints1);
            matchedPoints{2} = Ordered2dPoints(matchedPoints2);            
            matchedPoints{3} = Ordered2dPoints(matchedPoints3);
       
        end
    end
end
