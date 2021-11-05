classdef TwoViewProblem
    %TWOVIEWPROBLEM Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        dataPixelDomain
        dataHartleyDomain
        dataWorldDomain
        
        hartleyNmats
        poses
        calibrations
        cameras
        
        labels
        valPoints
        images
        allpts
        ifDPCPc3
    end
    
    methods
        function obj = TwoViewProblem(points, images, varargin)
            % parse the input arguments
            p = inputParser;
            
            addRequired(p, 'points');
            addRequired(p, 'images');
            addParameter(p, 'poses', []);
            addParameter(p, 'calibrations', []);
            addParameter(p, 'labels', []);
            addParameter(p, 'F', []);
            addParameter(p, 'H', []);
            addParameter(p, 'valPoints', []);
            addParameter(p, 'computeF', false);
            addParameter(p, 'computeH', false);
            
            parse(p, points, images, varargin{:});
            
            
            obj.poses = p.Results.poses;
            obj.calibrations = p.Results.calibrations;
            if ~isempty(obj.poses) && ~isempty(obj.calibrations)
                obj.cameras = Camera.transformCams(obj.poses, obj.calibrations);
            end
            obj.labels = p.Results.labels;
            
            % pixel domain
            obj.images = images;
            obj.dataPixelDomain.points = points;
            obj.dataPixelDomain.F = p.Results.F;
            obj.dataPixelDomain.H = p.Results.H;
            obj.dataPixelDomain.epipolarEmbed = ...
                normc(TwoViewGeometry.epipolarEmbed(obj.dataPixelDomain.points));
            obj.dataPixelDomain.homographicEmbed = ...
                normc(TwoViewGeometry.homographicEmbed(obj.dataPixelDomain.points));
            
            if ~isempty(obj.poses)
                obj.dataWorldDomain.E = EssenMat('absPose', obj.poses);
                obj.dataPixelDomain.F = ...
                    obj.dataWorldDomain.E.transform(obj.calibrations{1}, obj.calibrations{2});
            end
            
            % compute F from validation points
            obj.valPoints = p.Results.valPoints;
            if p.Results.computeF
                assert(~isempty(obj.valPoints));
                F_val = TwoViewProblem.computeFfromValPoints(obj.valPoints);
                if ~isempty(obj.dataPixelDomain.F)
                    disp('comparing F_GT with F_VAL')
                    obj.dataPixelDomain.F.angularError(F_val)
                else
                    obj.dataPixelDomain.F = F_val;
                end
            end
            
            % compute H from validation points
            if p.Results.computeH
                assert(~isempty(obj.valPoints));
                H_val = TwoViewProblem.computeHfromValPoints(obj.valPoints);
                if ~isempty(obj.dataPixelDomain.H)
                    disp('comparing H_GT with H_VAL')
                    obj.dataPixelDomain.H.angularError(H_val)
                else
                    obj.dataPixelDomain.H = H_val;
                end
            end
            
            % hartley domain
            for i = 1:2
                [Nmat, pointsOut] = points{i}.hartleyNormalize();
                obj.dataHartleyDomain.points{i} = pointsOut;
                obj.hartleyNmats{i} = Nmat;
            end
            
            obj.dataHartleyDomain.epipolarEmbed = ...
                normc(TwoViewGeometry.epipolarEmbed(obj.dataHartleyDomain.points));
            obj.dataHartleyDomain.homographicEmbed = ...
                normc(TwoViewGeometry.homographicEmbed(obj.dataHartleyDomain.points));
            
            if ~isempty(obj.dataPixelDomain.F)
                obj.dataHartleyDomain.F = ...
                    obj.dataPixelDomain.F.transform(obj.hartleyNmats{1}, obj.hartleyNmats{2});
            end
            if ~isempty(obj.dataPixelDomain.H)
                obj.dataHartleyDomain.H = ...
                    obj.dataPixelDomain.H.transform(obj.hartleyNmats{1}, obj.hartleyNmats{2});
            end
            
            % world domain
            if ~isempty(obj.calibrations)
                for i = 1:2
                    obj.dataWorldDomain.points{i} = ...
                        points{i}.invTransform(obj.calibrations{i});
                end
                obj.dataWorldDomain.epipolarEmbed = ...
                    normc(TwoViewGeometry.epipolarEmbed(obj.dataWorldDomain.points));
                obj.dataWorldDomain.homographicEmbed = ...
                    normc(TwoViewGeometry.homographicEmbed(obj.dataWorldDomain.points));
            end
        end

        function [metric] = goFund(obj, algorithm)
            [algoOut] = algorithm.runFund(obj);
            metric = struct();
            if isfield(algoOut, 'Obj')
                metric.Obj = algoOut.Obj;
            end
            if isfield(algoOut, 'n_inlier')
                metric.n_inlier = algoOut.n_inlier;
            end
            metric.time = algoOut.time;
            metric.iter = algoOut.iter;
            
            F_hartley = FundMat(reshape(algoOut.B, [3, 3]));
            metric.angFundErr = F_hartley.angularError(obj.dataHartleyDomain.F);
            
            F_pixel = F_hartley.invTransform(obj.hartleyNmats{1}, obj.hartleyNmats{2});
            if ~isempty(obj.valPoints)
                metric.sampsonErr = mean(F_pixel.sampsonError(obj.valPoints))^0.5;
            end
            
            if ~isempty(obj.calibrations)
                E_world = F_pixel.invTransform(obj.calibrations{1}, obj.calibrations{2});
                posesEst = E_world.toPoses();
                
                if ~isempty(obj.poses)
                    % compute ground truth relative pose
                    [~, poseRelGT] = Pose.normalizePoses(obj.poses);
                    [poseOut] = Pose.pickPosesByCmp(posesEst, poseRelGT{2});
                    [angRot, angTrans] = poseOut.angularError(poseRelGT{2});
                    
                    metric.angRot = angRot;
                    metric.angTrans = angTrans;
                end
            end
        end
        
        function [metric] = goHomoC1(obj, algorithm)
            [algoOut] = algorithm.runHomoC1(obj);
            
            metric = struct();
            metric.time = algoOut.time;
            metric.iter = algoOut.iter;
            if isfield(algoOut, 'n_inlier')
                metric.n_inlier = algoOut.n_inlier;
            end
            
            H_hartley = HomoMat(reshape(algoOut.B, [3, 3])');
            if ~isempty(obj.dataPixelDomain.H)
                metric.angHomoErr = H_hartley.angularError(obj.dataHartleyDomain.H);
            end
            
            H_pixel = H_hartley.invTransform(obj.hartleyNmats{1}, obj.hartleyNmats{2});
            if ~isempty(obj.valPoints)
                metric.transferErr = mean(H_pixel.transferError(obj.valPoints));
            end
            
            if ~isempty(obj.calibrations)
                H_world = H_pixel.invTransform(obj.calibrations{1}, obj.calibrations{2});
                
                posesEst = H_world.toPoses();

                if ~isempty(obj.poses)
                    % compute ground truth relative pose
                    [~, poseRelGT] = Pose.normalizePoses(obj.poses);
                    [poseOut] = Pose.pickPosesByCmp(posesEst, poseRelGT{2});
                    [angRot, angTrans] = poseOut.angularError(poseRelGT{2});

                    metric.angRot = angRot;
                    metric.angTrans = angTrans;
                end
            end
        end
        
        function [metric] = goHomoC3(obj, algorithm)
            [algoOut] = algorithm.runHomoC3(obj);
            
            metric = struct();
            metric.time = algoOut.time;
            metric.iter = algoOut.iter;
            if isfield(algoOut, 'n_inlier')
                metric.n_inlier = algoOut.n_inlier;
            end
            
            H_hartley = HomoMat.fromSubspace(algoOut.B);
            if ~isempty(obj.dataPixelDomain.H)
                metric.angHomoErr = H_hartley.angularError(obj.dataHartleyDomain.H);
            end
            
            H_pixel = H_hartley.invTransform(obj.hartleyNmats{1}, obj.hartleyNmats{2});
            if ~isempty(obj.valPoints)
                metric.transferErr = mean(H_pixel.transferError(obj.valPoints));
            end
            
            if ~isempty(obj.calibrations)
                H_world = H_pixel.invTransform(obj.calibrations{1}, obj.calibrations{2});
                
                posesEst = H_world.toPoses();

                if ~isempty(obj.poses)
                    % compute ground truth relative pose
                    [~, poseRelGT] = Pose.normalizePoses(obj.poses);
                    [poseOut] = Pose.pickPosesByCmp(posesEst, poseRelGT{2});
                    [angRot, angTrans] = poseOut.angularError(poseRelGT{2});

                    metric.angRot = angRot;
                    metric.angTrans = angTrans;
                end
            end
        end
    end
    
    methods(Static)
        function [F_pixel] = computeFfromValPoints(valPoints)
            assert(~isempty(valPoints));
            
            for i = 1:2
                [Nmat, pointsOut] = valPoints{i}.hartleyNormalize();
                vPHartley{i} = pointsOut;
                vPHartleyNmats{i} = Nmat;
            end
            
            epiEmbed = normc(TwoViewGeometry.epipolarEmbed(vPHartley));
            algo = AlgoSVD;
            [algoOut] = algo.run(epiEmbed);
            F_hartley = FundMat(reshape(algoOut.B, [3, 3]));
            F_pixel = F_hartley.invTransform(vPHartleyNmats{1}, vPHartleyNmats{2});
        end
        
        function [H_pixel] = computeHfromValPoints(valPoints)
            assert(~isempty(valPoints));
            
            for i = 1:2
                [Nmat, pointsOut] = valPoints{i}.hartleyNormalize();
                vPHartley{i} = pointsOut;
                vPHartleyNmats{i} = Nmat;
            end
            
            homoEmbed = normc(TwoViewGeometry.homographicEmbed(vPHartley));
            algo = AlgoSVD;
            [algoOut] = algo.run(homoEmbed);
            H_hartley = HomoMat(reshape(algoOut.B, [3, 3])');
            H_pixel = H_hartley.invTransform(vPHartleyNmats{1}, vPHartleyNmats{2});
        end
    end
end

