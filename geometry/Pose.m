classdef Pose < Camera
    %POSE Summary of this class goes here
    %   Detailed explanation goes here
    
    
    methods
        function obj = Pose(R_t)
            assert(all(size(R_t) == [3, 4]));
            R = R_t(:, 1:3);
            assert(norm(R'*R-eye(3), 'fro') < 1e-3);
            obj@Camera(R_t);
        end
        
        function [angRot, angTrans] = angularError(obj, other)
            R_1 = obj.Pmat(:, 1:3);
            t_1 = obj.Pmat(:, 4);
            R_2 = other.Pmat(:, 1:3);
            t_2 = other.Pmat(:, 4);
            
            % determine angle difference between rotations
            angRot = abs(acosd((trace(R_1.'*R_2) - 1)/2));
            
            % determine angle difference between translations
            angTrans = abs(acosd(dot(t_2/norm(t_2), t_1/norm(t_1))));
        end
        
        function [eucTrans] = eucTransError(obj, other)
            t_1 = obj.Pmat(:, 4);
            t_2 = other.Pmat(:, 4);
            eucTrans = norm(t_1-t_2);
        end
    end
    
    methods(Static)
        function [H4, posesOut] = normalizePoses(poses)
            nViews = length(poses);
            
            pose1 = poses{1}.Pmat;
            H4 = [pose1(:, 1:3)', -pose1(:, 1:3)' * pose1(:, 4); ...
                zeros(1, 3), 1];
            
            posesOut = cell(1, nViews);
            for i = 1:nViews
                posesOut{i} = Pose(poses{i}.Pmat*H4);
            end
            
            assert(norm(posesOut{1}.Pmat-eye(3, 4), 'fro') < 1e-3);
        end
        
        function [poseOut, poseOutId] = pickPosesByCmp(poses, poseCmp)
            nPoses = length(poses);
            assert(nPoses >= 1);
            
            minErr = inf;
            poseOut = poses{1};
            poseOutId = 1;
            for i = 1:nPoses
                [angRot, angTrans] = poses{i}.angularError(poseCmp);
                if angRot + angTrans < minErr
                    poseOut = poses{i};
                    minErr = angRot + angTrans;
                    poseOutId = i;
                end
            end
        end
    end
end
