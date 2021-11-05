classdef TwoViewGeometry < MultipleViewGeometry
    %TWOVIEWGEOMETRY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = TwoViewGeometry(inputArg1, inputArg2)
            %TWOVIEWGEOMETRY Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
    end
    
    methods(Static)
        function [embed] = epipolarEmbed(points)
            nViews = length(points);
            assert(nViews == 2);
            
            embed = zeros(9, points{1}.length());
            for i = 1:points{1}.length()
                x1 = points{1}.points(:, i);
                x2 = points{2}.points(:, i);
                embed(:, i) = kron(x1, x2);
            end
        end
        
        function [embed] = homographicEmbed(points)
            nViews = length(points);
            assert(nViews == 2);
            
            pts1 = points{1}.points(1:2, :);
            pts2 = points{2}.points(1:2, :);
            
            n = size(pts1, 2);
            embed = zeros(2*n, 9);
            embed(1:2:2*n, 1:2) = pts1';
            embed(1:2:2*n, 3) = 1;
            embed(2:2:2*n, 4:5) = pts1';
            embed(2:2:2*n, 6) = 1;
            x1 = pts1(1, :)';
            y1 = pts1(2, :)';
            x2 = pts2(1, :)';
            y2 = pts2(2, :)';
            embed(1:2:2*n, 7) = -x2 .* x1;
            embed(2:2:2*n, 7) = -y2 .* x1;
            embed(1:2:2*n, 8) = -x2 .* y1;
            embed(2:2:2*n, 8) = -y2 .* y1;
            embed(1:2:2*n, 9) = -x2;
            embed(2:2:2*n, 9) = -y2;
            
            embed = embed';
        end
        
        function visCorresp(images, points, idPoints)
            if nargin == 2
                idPoints = 1:points{1}.length();
            end
            TwoViewGeometry.visCorrespWithScore(images, points, ...
                ones(1, points{1}.length()), [0, 1], idPoints);
        end
        
        function visCorrespWithScore(images, points, scores, scoreRange, idPoints)
            % optional idPoints specifying the ids of points to be
            % visualized.
            assert(length(points) == 2);
            assert(length(images) == 2);
            assert(points{1}.length() == length(scores));
            
            function [j] = rangeTo256(x, scoreRange)
                j = round(min([256, ...
                    (255 * x + scoreRange(2) - 256 * scoreRange(1)) / (scoreRange(2) - scoreRange(1))]));
            end
            
            [imH, imW, imC] = size(images{1});
            
            figure;
            subplot(2, 1, 1);
            montage(images, 'ThumbnailSize', []);
            subplot(2, 1, 2);
            montage(images, 'ThumbnailSize', []);
            hold on;
            
            %             scoresNormalized = normalize(scores, 'range', scoreRange);
            cmap = colormap;
            getColor = @(i) cmap(rangeTo256(scores(i), scoreRange), :);
            
            if nargin == 4
                idPoints = 1:points{1}.length();
            else
                if iscolumn(idPoints)
                    idPoints = idPoints';
                end
            end
            for i = idPoints
                p1 = points{1}.points(1:2, i);
                p2 = points{2}.points(1:2, i) + [imW; 0];
                plot(p1(1), p1(2), 'ro', 'Color', 'yellow');
                plot(p2(1), p2(2), 'ro', 'Color', 'yellow');
                line([p1(1), p2(1)], [p1(2), p2(2)], 'Color', ...
                    getColor(i), 'LineWidth', 1);
            end
            colorbar;
        end
    end
end
