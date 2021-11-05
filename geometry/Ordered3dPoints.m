classdef Ordered3dPoints
    %ORDEREDPOINTS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points
    end
    
    methods
        function obj = Ordered3dPoints(points)
            switch size(points, 1)
                case 3
                    obj.points = points;
                    obj.points(4, :) = 1;
                case 4
                    obj.points = points ./ points(4, :);
                otherwise
                    error('wrong dimension of 3d points')
            end
        end
        
        function nPoints = length(obj)
            nPoints = size(obj.points, 2);
        end
        
        function err = SqError(obj, other)
            err = sum((obj.points - other.points).^2, 1);
        end
        
        function pointsOut = transform(obj, P)
            assert(all(size(P) == [4, 4]));
            pointsOut = Ordered3dPoints(P*obj.points);
        end
        
        function pointsOut = invTransform(obj, P)
            assert(all(size(P) == [4, 4]));
            pointsOut = Ordered3dPoints(P \ obj.points);
        end
        
        function pointsOut = project(obj, P)
            pointsOut = Ordered2dPoints(P.Pmat*obj.points);
        end
        
        function [Nmat, pointsOut] = hartleyNormalize(obj)
            points0 = mean(obj.points, 2);
            norm0 = sqrt(mean(sum((obj.points - points0).^2, 1)));
            Nmat = diag([sqrt(3) / norm0; sqrt(3) / norm0; sqrt(3) / norm0; 1]);
            Nmat(1:3, 4) = -sqrt(3) * points0(1:3) / norm0;
            
            pointsOut = obj.transform(Nmat);
        end
        
        function [Nmat, pointsOut] = rickNormalize(obj)
            points0 = median(obj.points, 2);
            norm0 = 1.4826*median(sum(abs(obj.points - points0), 1));
%             norm0 = sqrt(mean(sum((obj.points - points0).^2, 1)));

            Nmat = diag([sqrt(3) / norm0; sqrt(3) / norm0; sqrt(3) / norm0; 1]);
            Nmat(1:3, 4) = -sqrt(3) * points0(1:3) / norm0;
            
            pointsOut = obj.transform(Nmat);
        end
        
        function pointsOut = filter(obj, label)
            pointsOut = Ordered3dPoints(obj.points(:, label));
        end
    end
end
