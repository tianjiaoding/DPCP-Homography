classdef ThreeViewProblem
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
    end
    
    methods
        function obj = ThreeViewProblem(points, images, varargin)
            % parse the input arguments
            p = inputParser;
            
            addRequired(p, 'points');
            addRequired(p, 'images');
            addParameter(p, 'poses', []);
            addParameter(p, 'calibrations', []);
            addParameter(p, 'labels', []);
            addParameter(p, 'T', []);
            addParameter(p, 'H', []);
            addParameter(p, 'valPoints', []);
            addParameter(p, 'computeT', false);
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
            obj.dataPixelDomain.T = p.Results.T;
            obj.dataPixelDomain.H = p.Results.H;
            
            obj.dataPixelDomain.trifocalEmbed = ...
                normc(ThreeViewGeometry.trifocalEmbed(obj.dataPixelDomain.points));
            obj.dataPixelDomain.homographicEmbed = ...
                normc(ThreeViewGeometry.homographicEmbed(obj.dataPixelDomain.points));
            
            if ~isempty(obj.poses)
                obj.dataWorldDomain.Tcal = TrifoTensor.fromCameras(obj.poses);
                obj.dataPixelDomain.T = ...
                    obj.dataWorldDomain.Tcal.transform(obj.calibrations{1}, obj.calibrations{2}, obj.calibrations{3});
                % this should be equivalent to
                TrifoTensor.fromCameras(obj.cameras);
                obj.dataPixelDomain.T;
            end
            
            %             % compute F from validation points
            %             obj.valPoints = p.Results.valPoints;
            %             if p.Results.computeT
            %                 assert(~isempty(obj.valPoints));
            %                 F_val = TwoViewProblem.computeFfromValPoints(obj.valPoints);
            %                 if ~isempty(obj.dataPixelDomain.F)
            %                     disp('comparing F_GT with F_VAL')
            %                     obj.dataPixelDomain.F.angularError(F_val)
            %                 else
            %                     obj.dataPixelDomain.F = F_val;
            %                 end
            %             end
            %
            %             % compute H from validation points (add by Xinyue)
            %             if p.Results.computeH
            %                 assert(~isempty(obj.valPoints));
            %                 H_val = TwoViewProblem.computeHfromValPoints(obj.valPoints);
            %                 if ~isempty(obj.dataPixelDomain.F)
            %                     disp('comparing H_GT with H_VAL')
            %                     obj.dataPixelDomain.H.angularError(H_val)
            %                 else
            %                     obj.dataPixelDomain.H = H_val;
            %                 end
            %             end
            
            % hartley domain
            for i = 1:3
                [Nmat, pointsOut] = points{i}.hartleyNormalize();
                obj.dataHartleyDomain.points{i} = pointsOut;
                obj.hartleyNmats{i} = Nmat;
            end
            
            obj.dataHartleyDomain.trifocalEmbed = ...
                normc(ThreeViewGeometry.trifocalEmbed(obj.dataHartleyDomain.points));
            obj.dataHartleyDomain.homographicEmbed = ...
                normc(ThreeViewGeometry.homographicEmbed(obj.dataHartleyDomain.points));
            
            if ~isempty(obj.dataPixelDomain.T)
                obj.dataHartleyDomain.T = ...
                    obj.dataPixelDomain.T.transform(obj.hartleyNmats{1}, obj.hartleyNmats{2}, obj.hartleyNmats{3});
            end
            if ~isempty(obj.dataPixelDomain.H)
                obj.dataHartleyDomain.H = ...
                    obj.dataPixelDomain.H.transform(obj.hartleyNmats{1}, obj.hartleyNmats{2}, obj.hartleyNmats{3});
            end
            
            % world domain
            if ~isempty(obj.calibrations)
                for i = 1:3
                    obj.dataWorldDomain.points{i} = ...
                        points{i}.invTransform(obj.calibrations{i});
                end
                obj.dataWorldDomain.trifocalEmbed = ...
                    normc(ThreeViewGeometry.trifocalEmbed(obj.dataWorldDomain.points));
                obj.dataWorldDomain.homographicEmbed = ...
                    normc(ThreeViewGeometry.homographicEmbed(obj.dataWorldDomain.points));
            end
        end
        
        function [metric] = goTrifo(obj, algorithm)
            [algoOut] = algorithm.runTrifo(obj);
            metric = struct();
            if isfield(algoOut, 'Obj')
                metric.Obj = algoOut.Obj;
            end
            if isfield(algoOut, 'n_inlier')
                metric.n_inlier = algoOut.n_inlier;
            end
            metric.time = algoOut.time;
            metric.iter = algoOut.iter;
            
            T_hartley = TrifoTensor(reshape(algoOut.B, [3, 3, 3]));
            metric.angTrifoErr = T_hartley.angularError(obj.dataHartleyDomain.T);
            
            T_pixel = T_hartley.invTransform(obj.hartleyNmats{1}, obj.hartleyNmats{2}, obj.hartleyNmats{3});
            
            if ~isempty(obj.valPoints)
                metric.sampsonErr = mean(T_pixel.sampsonError(obj.valPoints))^0.5;
            end
            
            if ~isempty(obj.calibrations)
                Tcal_world = T_pixel.invTransform(obj.calibrations{1}, obj.calibrations{2}, obj.calibrations{3});
                [posesEst12, posesEst13] = Tcal_world.toPoses();
                
                if ~isempty(obj.poses)
                    % compute ground truth relative pose
                    [~, poseRelGT] = Pose.normalizePoses(obj.poses);
                    [poseOut12] = Pose.pickPosesByCmp(posesEst12, poseRelGT{2});
                    [poseOut13] = Pose.pickPosesByCmp(posesEst13, poseRelGT{3});
                    [angRot12, angTrans12] = poseOut12.angularError(poseRelGT{2});
                    [angRot13, angTrans13] = poseOut13.angularError(poseRelGT{3});
                    poseEst = poseRelGT;
                    poseEst{2} = poseOut12;
                    poseEst{3} = poseOut13;
                    Estcameras = Camera.transformCams(poseEst, obj.calibrations);
                    score = ThreeViewGeometry.reprojError(Estcameras, obj.dataPixelDomain.points);
                    
                    metric.angRot12 = angRot12;
                    metric.angTrans12 = angTrans12;
                    metric.angRot13 = angRot13;
                    metric.angTrans13 = angTrans13;
                    metric.angRot = (angRot12 + angRot13) / 2;
                    metric.angTrans = (angTrans12 + angTrans13) / 2;
                end
            end
        end
        
        function [metric] = goTriHomo(obj, algorithm)
            [algoOut] = algorithm.runTriHomo(obj);
            metric = struct();
            if isfield(algoOut, 'Obj')
                metric.Obj = algoOut.Obj;
            end
            if isfield(algoOut, 'n_inlier')
                metric.n_inlier = algoOut.n_inlier;
            end
            metric.time = algoOut.time;
            metric.iter = algoOut.iter;
            metric.LOiter = algoOut.LOiter;
            
            H_hartley = HomoTensor(reshape(algoOut.B, [3, 3, 3]));
%             metric.angTrifoErr = T_hartley.angularError(obj.dataHartleyDomain.T);
            H_pixel = H_hartley.invTransform(obj.hartleyNmats{1}, obj.hartleyNmats{2}, obj.hartleyNmats{3});
            
            if ~isempty(obj.valPoints)
                metric.transErr = mean(H_pixel.transferError(obj.dataPixelDomain.points));
            end
            
            if ~isempty(obj.calibrations)
                Hcal_world = H_pixel.invTransform(obj.calibrations{1}, obj.calibrations{2}, obj.calibrations{3});
%                 [posesEst12, posesEst13] = Hcal_world.toPoses();
                
                if ~isempty(obj.poses)
                    [~, poseRelGT] = Pose.normalizePoses(obj.poses);
%                     H4 = [obj.poses{1}.Pmat(:, 1:3)', -obj.poses{1}.Pmat(:, 1:3)' * obj.poses{1}.Pmat(:, 4); zeros(1, 3), 1];
                    % compute ground truth relative pose
                    [H21, H31] = ABfromHTensor(Hcal_world.Htsr);
                    H12_de = inv(H21);
                    H13_de = inv(H31);
%                     [R2, t2] = decompose_homo(H12_de, obj.poses{2}.Pmat*H4);
%                     [R3, t3] = decompose_homo(H13_de, obj.poses{3}.Pmat*H4);
                    [R2, t2] = decompose_homo(H12_de, poseRelGT{2}.Pmat);
                    [R3, t3] = decompose_homo(H13_de, poseRelGT{3}.Pmat);
                    poseOut12 = Pose([R2, t2]);
                    poseOut13 = Pose([R3, t3]);
                    [angRot12, angTrans12] = poseOut12.angularError(poseRelGT{2});
                    [angRot13, angTrans13] = poseOut13.angularError(poseRelGT{3});
                    poseEst = poseRelGT;
                    poseEst{2} = poseOut12;
                    poseEst{3} = poseOut13;
                    Estcameras = Camera.transformCams(poseEst, obj.calibrations);
%                     score = ThreeViewGeometry.reprojError(Estcameras, obj.dataPixelDomain.points);
                    
                    metric.angRot12 = angRot12;
                    metric.angTrans12 = angTrans12;
                    metric.angRot13 = angRot13;
                    metric.angTrans13 = angTrans13;
                    metric.angRot = (angRot12 + angRot13) / 2;
                    metric.angTrans = (angTrans12 + angTrans13) / 2;
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
            H_hartley = HomoMat(reshape(algoOut.B, [3, 3]));
            H_pixel = H_hartley.invTransform(vPHartleyNmats{1}, vPHartleyNmats{2});
        end
    end
end

function [A] = computeA(p)
x1 = p(1);
y1 = p(2);
x2 = p(3);
y2 = p(4);
x3 = p(5);
y3 = p(6);

A(1, :) = [x1, 0, -x1 * x2, 0, 0, 0, -x1 * x3, 0, x1 * x2 * x3, ...
    y1, 0, -x2 * y1, 0, 0, 0, -x3 * y1, 0, x2 * x3 * y1, ...
    1, 0, -x2, 0, 0, 0, -x3, 0, x2 * x3];
A(2, :) = [0, x1, -x1 * y2, 0, 0, 0, 0, -x1 * x3, x1 * x3 * y2, ...
    0, y1, -y1 * y2, 0, 0, 0, 0, -x3 * y1, x3 * y1 * y2, ...
    0, 1, -y2, 0, 0, 0, 0, -x3, x3 * y2];
A(3, :) = [0, 0, 0, x1, 0, -x1 * x2, -x1 * y3, 0, x1 * x2 * y3, ...
    0, 0, 0, y1, 0, -x2 * y1, -y1 * y3, 0, x2 * y1 * y3, ...
    0, 0, 0, 1, 0, -x2, -y3, 0, x2 * y3];
A(4, :) = [0, 0, 0, 0, x1, -x1 * y2, 0, -x1 * y3, x1 * y2 * y3, ...
    0, 0, 0, 0, y1, -y1 * y2, 0, -y1 * y3, y1 * y2 * y3, ...
    0, 0, 0, 0, 1, -y2, 0, -y3, y2 * y3];
end