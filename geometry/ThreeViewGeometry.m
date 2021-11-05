classdef ThreeViewGeometry < MultipleViewGeometry
    %TWOVIEWGEOMETRY Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Property1
    end
    
    methods
        function obj = ThreeViewGeometry(inputArg1, inputArg2)
            %TWOVIEWGEOMETRY Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        
    end
    
    methods(Static)
        function [embed] = trifocalEmbed(points)
            
            nViews = length(points);
            assert(nViews == 3);
            
            p1 = points{1}.points(1:2, :);
            p2 = points{2}.points(1:2, :);
            p3 = points{3}.points(1:2, :);
            
            A = zeros(4*points{1}.length(), 27);
            
            for i = 1:points{1}.length()
                x1 = p1(1, i);
                y1 = p1(2, i);
                x2 = p2(1, i);
                y2 = p2(2, i);
                x3 = p3(1, i);
                y3 = p3(2, i);
                
                A(4*(i - 1)+1, :) = [x1, 0, -x1 * x2, 0, 0, 0, -x1 * x3, 0, x1 * x2 * x3, ...
                    y1, 0, -x2 * y1, 0, 0, 0, -x3 * y1, 0, x2 * x3 * y1, ...
                    1, 0, -x2, 0, 0, 0, -x3, 0, x2 * x3];
                A(4*(i - 1)+2, :) = [0, x1, -x1 * y2, 0, 0, 0, 0, -x1 * x3, x1 * x3 * y2, ...
                    0, y1, -y1 * y2, 0, 0, 0, 0, -x3 * y1, x3 * y1 * y2, ...
                    0, 1, -y2, 0, 0, 0, 0, -x3, x3 * y2];
                A(4*(i - 1)+3, :) = [0, 0, 0, x1, 0, -x1 * x2, -x1 * y3, 0, x1 * x2 * y3, ...
                    0, 0, 0, y1, 0, -x2 * y1, -y1 * y3, 0, x2 * y1 * y3, ...
                    0, 0, 0, 1, 0, -x2, -y3, 0, x2 * y3];
                A(4*(i - 1)+4, :) = [0, 0, 0, 0, x1, -x1 * y2, 0, -x1 * y3, x1 * y2 * y3, ...
                    0, 0, 0, 0, y1, -y1 * y2, 0, -y1 * y3, y1 * y2 * y3, ...
                    0, 0, 0, 0, 1, -y2, 0, -y3, y2 * y3];
            end
            
            embed = A';
            assert(all(size(embed) == [27, 4 * points{1}.length()]))
        end
        
        function [embed] = homographicEmbed(points)
            nViews = length(points);
            assert(nViews == 3);
            
            p1 = points{1}.points;
            p2 = points{2}.points;
            p3 = points{3}.points;
            
            A = [];
            
            for i = 1:points{1}.length()
                pts = {p1(:, i), p2(:, i), p3(:, i)};
                for idPoint = 1:3
                    for idStdBasis = 1:3
                        e = zeros(3, 1);
                        e(idStdBasis) = 1;
                        pts_ = pts;
                        pts_{idPoint} = e;
                        A = [A, ThreeViewGeometry.computeHEmbedding(pts_{1}, pts_{2}, pts_{3})];
                    end
                end
            end
            
            embed = A;
            assert(all(size(embed) == [27, 9 * points{1}.length()]))
        end
        
        function visCorresp(images, points, idPoints)
            error('unimplemented')
            if nargin == 2
                idPoints = 1:points{1}.length();
            end
            TwoViewGeometry.visCorrespWithScore(images, points, ...
                ones(1, points{1}.length()), [0, 1], idPoints);
        end
        
        function visCorrespWithScore(images, points, scores, scoreRange, idPoints)
            error('unimplemented')
        end
        function visCorrespWithoutLine(images, points, scores, scoreRange, idPoints)
            % optional idPoints specifying the ids of points to be
            % visualized.
            assert(length(points) == 3);
            assert(length(images) == 3);
            assert(points{1}.length() == length(scores));
            
            function [j] = rangeTo256(x, scoreRange)
                j = round(min([256, ...
                    (255 * x + scoreRange(2) - 256 * scoreRange(1)) / (scoreRange(2) - scoreRange(1))]));
            end
            
            [imH, imW, imC] = size(images{1});
            
            figure;
%             montage(images);
            montage(images, 'ThumbnailSize', [], 'Size', [1,3]);
            hold on;
            
            %             scoresNormalized = normalize(scores, 'range', scoreRange);
%             cmap = colormap;
%             getColor = @(i) cmap(rangeTo256(scores(i), scoreRange), :);
%             
            if nargin == 4
                idPoints = 1:points{1}.length();
            else
                if iscolumn(idPoints)
                    idPoints = idPoints';
                end
            end
            for i = idPoints
                if scores(i) < scoreRange(2)
                    p1 = points{1}.points(1:2, i);
                    p2 = points{2}.points(1:2, i) + [imW; 0];
                    p3 = points{3}.points(1:2, i) + [2*imW; 0];
                    plot(p1(1), p1(2), 'ro', 'Color', 'yellow');
                    plot(p2(1), p2(2), 'ro', 'Color', 'yellow');
                    plot(p3(1), p3(2), 'ro', 'Color', 'yellow');
                else
                    p1 = points{1}.points(1:2, i);
                    p2 = points{2}.points(1:2, i) + [imW; 0];
                    p3 = points{3}.points(1:2, i) + [2*imW; 0];
                    plot(p1(1), p1(2), 'ro', 'Color', 'blue');
                    plot(p2(1), p2(2), 'ro', 'Color', 'blue');
                    plot(p3(1), p3(2), 'ro', 'Color', 'blue');
                end
            end
%             colorbar;
        end
    end
    
    methods(Static, Access = private)
        function [v] = computeHEmbedding(p, pp, ppp)
            E = zeros(3, 3, 3);
            for i = 1:3
                for j = 1:3
                    for k = 1:3
                        E(j, k, i) = p(i) * pp(j) * ppp(k);
                    end
                end
            end
            v = reshape(E, 27, 1);
        end
    end
end
