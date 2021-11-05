classdef Ordered2dPoints
    %ORDEREDPOINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points
    end
    
    methods
        function obj = Ordered2dPoints(points)
            switch size(points, 1)
                case 2
                    obj.points = points;
                    obj.points(3, :) = 1;
                case 3
                    obj.points = points ./ points(3, :);
                otherwise
                    error('wrong dimension of 2d points')
            end
        end
        
        function nPoints = length(obj)
            nPoints = size(obj.points, 2);
        end
        
        function err = SqError(obj, other)
            err = sum((obj.points - other.points).^2, 1);
        end
        
        function err = angularError(obj, other)
            pts1 = normc(obj.points);
            pts2 = normc(other.points);
            
            nPts = obj.length();
            err = zeros(1, nPts);
            for i=1:nPts
                err(i) = acosd(min(pts1(:, i)'*pts2(:, i), 1));
            end
        end
        
        function pointsOut = transform(obj, K)
            assert(all(size(K) == [3, 3]));
            pointsOut = Ordered2dPoints(K*obj.points);
        end
        
        function pointsOut = invTransform(obj, K)
            assert(all(size(K) == [3, 3]));
            pointsOut = Ordered2dPoints(K \ obj.points);
        end
        
        function [Nmat, pointsOut] = hartleyNormalize(obj)
            points0 = mean(obj.points, 2);
            norm0 = sqrt(mean(sum((obj.points - points0).^2, 1)));
            Nmat = diag([sqrt(2) / norm0; sqrt(2) / norm0; 1]);
            Nmat(1:2, 3) = -sqrt(2) * points0(1:2) / norm0;
            
            pointsOut = obj.transform(Nmat);
        end
        
        function [Nmat, pointsOut] = rickNormalize(obj)
            points0 = median(obj.points, 2);
            norm0 = 1.4826*median(sum(abs(obj.points - points0), 1));
%             norm0 = sqrt(mean(sum((obj.points - points0).^2, 1)));

            Nmat = diag([sqrt(2) / norm0; sqrt(2) / norm0; 1]);
            Nmat(1:2, 3) = -sqrt(2) * points0(1:2) / norm0;
            
            pointsOut = obj.transform(Nmat);
        end
        
        function pointsOut = filter(obj, label)
            pointsOut = Ordered2dPoints(obj.points(:, label));
        end
        
        function pointsOut = addNoise(obj, sigma)
            noise = sigma*randn(2, obj.length());
            pointsOut = Ordered2dPoints(obj.points(1:2, :) + noise);
        end
        
        function pointsOut = permute(obj, perm)
            pointsOut = Ordered2dPoints(obj.points(:, perm));
        end
    end
end
