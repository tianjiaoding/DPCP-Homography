classdef (Abstract) AlgoRPCA
    %ALGORPCA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (Abstract)
        params
    end
    
    methods (Abstract)
        run(obj)
    end
    
    methods
        function algoOut = runFund(obj, problem)
            obj.params.c = 1;
            obj.params.g = 1;
            obj.params.domain = 'Hartley';
            algoOut = obj.run(problem.dataHartleyDomain.epipolarEmbed);
        end
        
        function algoOut = runEssential(obj, problem)
            obj.params.c = 1;
            obj.params.g = 1;
            obj.params.domain = 'Hartley';
            NormalizedImage_points_1 = problem.dataPixelDomain.points{1,1}.points;
            NormalizedImage_points_1 = inv(problem.calibrations{1})*NormalizedImage_points_1;
            NormalizedImage_points_1 = NormalizedImage_points_1./NormalizedImage_points_1(3,:);
            NormalizedImage_points_2 = problem.dataPixelDomain.points{1,2}.points;
            NormalizedImage_points_2 = inv(problem.calibrations{2})*NormalizedImage_points_2;
            NormalizedImage_points_2 = NormalizedImage_points_2./NormalizedImage_points_2(3,:);
            epipolar_embedding = zeros(9,size(NormalizedImage_points_2,2));
            for i = 1:size(NormalizedImage_points_2,2)
                epipolar_embedding(:,i) = kron(NormalizedImage_points_1(:,i), NormalizedImage_points_2(:,i));
            end
            epipolar_embedding = normc(epipolar_embedding);
            algoOut = obj.run(epipolar_embedding);
        end
        
        function algoOut = runEssential5pt(obj, problem)
            obj.params.c = 4;
            obj.params.g = 1;
            obj.params.domain = 'Hartley';
            NormalizedImage_points_1 = problem.dataPixelDomain.points{1,1}.points;
            NormalizedImage_points_1 = inv(problem.calibrations{1})*NormalizedImage_points_1;
            NormalizedImage_points_1 = NormalizedImage_points_1./NormalizedImage_points_1(3,:);
            NormalizedImage_points_2 = problem.dataPixelDomain.points{1,2}.points;
            NormalizedImage_points_2 = inv(problem.calibrations{2})*NormalizedImage_points_2;
            NormalizedImage_points_2 = NormalizedImage_points_2./NormalizedImage_points_2(3,:);
            epipolar_embedding = zeros(9,size(NormalizedImage_points_2,2));
            for i = 1:size(NormalizedImage_points_2,2)
                epipolar_embedding(:,i) = kron(NormalizedImage_points_1(:,i), NormalizedImage_points_2(:,i));
            end
            epipolar_embedding = epipolar_embedding([1,4,7,2,5,8,3,6,9], :);
            epipolar_embedding = normc(epipolar_embedding);
            algoOut = obj.run(epipolar_embedding);
        end
        
        function algoOut = runEssential5pt_DPCPc3(obj, problem)
            obj.params.c = 4;
            obj.params.g = 1;
            obj.params.domain = 'Hartley';
            NormalizedImage_points_1 = problem.dataPixelDomain.points{1,1}.points;
            NormalizedImage_points_1 = inv(problem.calibrations{1})*NormalizedImage_points_1;
            NormalizedImage_points_1 = NormalizedImage_points_1./NormalizedImage_points_1(3,:);
            NormalizedImage_points_2 = problem.dataPixelDomain.points{1,2}.points;
            NormalizedImage_points_2 = inv(problem.calibrations{2})*NormalizedImage_points_2;
            NormalizedImage_points_2 = NormalizedImage_points_2./NormalizedImage_points_2(3,:);
            epipolar_embedding = zeros(9,size(NormalizedImage_points_2,2));
            for i = 1:size(NormalizedImage_points_2,2)
                epipolar_embedding(:,i) = kron(NormalizedImage_points_1(:,i), NormalizedImage_points_2(:,i));
            end
            epipolar_embedding = epipolar_embedding([1,4,7,2,5,8,3,6,9], :);
            epipolar_embedding = normc(epipolar_embedding);
            [~, B, ~, time, ~, iter] = DPCP_IRLS_G_2tb...
                (epipolar_embedding, ...
                3, ...
                1e-9,...
                200,...
                1e-7, ...
                1, ...
                inf, inf);
            invcos_weight = 1./(vecnorm(B'*epipolar_embedding)+1e-10);
            epipolar_embedding = invcos_weight.*epipolar_embedding;
            algoOut = obj.run(epipolar_embedding);
            algoOut.weight = invcos_weight;

        end
        
        function algoOut = runFund_pixel(obj, problem)
            obj.params.c = 1;
            obj.params.g = 1;
            obj.params.domain = 'Pixel';
            algoOut = obj.run(problem.dataPixelDomain.epipolarEmbed);
        end
        
        function algoOut = runHomoC1(obj, problem)
            obj.params.c = 1;
            obj.params.g = 2;
            obj.params.domain = 'Hartley';
            algoOut = obj.run(problem.dataHartleyDomain.homographicEmbed);
        end
        
        function algoOut = runHomoC1_pixel(obj, problem)
            obj.params.c = 1;
            obj.params.g = 2;
            obj.params.domain = 'Pixel';
            algoOut = obj.run(problem.dataPixelDomain.homographicEmbed);
        end
        
        function algoOut = runHomoC3(obj, problem)
            obj.params.c = 3;
            obj.params.g = 1;
            algoOut = obj.run(problem.dataHartleyDomain.epipolarEmbed);
        end
        
        function algoOut = runRoadPlane(obj, problem)
            obj.params.c = 1;
            obj.params.g = 1;
            algoOut = obj.run(problem.points_homo);
        end
        
        function algoOut = runMultibody(obj, problem)
            obj.params.c = 1;
            obj.params.g = 1;
            obj.params.domain = 'Pixel';
            algoOut = obj.run(problem.VeroneseEmbed);
        end
        
        function algoOut = runTrifo(obj, problem)
            obj.params.c = 1;
            obj.params.g = 4;
            obj.params.domain = 'Hartley';
            algoOut = obj.run(problem.dataHartleyDomain.trifocalEmbed);
        end
        
        function algoOut = runTriHomo(obj, problem)
            obj.params.c = 1;
            obj.params.g = 9;
            obj.params.domain = 'Hartley';
            algoOut = obj.run(problem.dataHartleyDomain.homographicEmbed);
%             algoOut = obj.run(problem.dataPixelDomain.homographicEmbed);
        end
    end
end

