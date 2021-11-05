classdef MultipleViewGeometry
    %TWOVIEWGEOMETRY Summary of this class goes here
    %   Detailed explanation goes here
    
    methods
        function obj = MultipleViewGeometry(inputArg1, inputArg2)
            %TWOVIEWGEOMETRY Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        
    end
    
    methods(Static)
        function points3D = triangulate(cameras, points)
            nViews = length(cameras);
            assert(nViews == length(points));
            assert(nViews >= 2);
            
            nPoints = points{1}.length();
            pointsMatrix = zeros(3*nViews, nPoints);
            Pmats = cell(1, nViews);
            for i = 1:nViews
                pointsMatrix(3*i-2:3*i, :) = points{i}.points;
                Pmats{i} = cameras{i}.Pmat;
            end
            points3D = triangulation3D(Pmats, pointsMatrix);
        end
        
        function [err] = reprojError(cameras, points, points3D)
            nViews = length(cameras);
            assert(nViews == length(points));
            assert(nViews >= 2);
            
            nPoints = points{1}.length();
            if nargin ~= 3
                points3D = MultipleViewGeometry.triangulate(cameras, points);
            end
            
            ErrMat = zeros(nViews, nPoints);
            
            for i = 1:nViews
                pointsEst = Ordered2dPoints(cameras{i}.Pmat*points3D);
                ErrMat(i, :) = pointsEst.SqError(points{i});
            end
            
            % average squared error, e.g., for the i-th point,
            % the error is 0.5*(d^2(x_hat-x1)+d^2(x_hat-x2)),
            % where x1, x2 is the i-th point in view 1,2 respectively.
            err = sum(ErrMat, 1) ./ nViews;
        end
        
        function [labels] = labelCorrespByPose(cameras, points, th)
            [err] = MultipleViewGeometry.reprojError(cameras, points);
            labels = err <= th^2;
        end
    end
end
