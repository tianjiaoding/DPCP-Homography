classdef Camera
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Pmat
    end
    
    methods
        function obj = Camera(Pmat)
            assert(all(size(Pmat) == [3, 4]));
            obj.Pmat = Pmat;
        end
        
        function CamOut = transform(obj, K)
            assert(all(size(K) == [3, 3]));
            CamOut = Camera(K*obj.Pmat);
        end
        
        function CamOut = invTransform(obj, K)
            assert(all(size(K) == [3, 3]));
            CamOut = Camera(K \ obj.Pmat);
        end
        
        function CamOut = transform23(obj, K2, K3)
            assert(all(size(K2) == [3, 3]));
            assert(all(size(K3) == [4, 4]));
            CamOut = Camera(K2*obj.Pmat/K3);
        end
        
        function CamOut = invTransform23(obj, K2, K3)
            assert(all(size(K2) == [3, 3]));
            assert(all(size(K3) == [4, 4]));
            CamOut = Camera(K2 \ obj.Pmat*K3);
        end
        
        function [Pvec] = toVec(obj)
            Pvec = Camera.Pmat2vec(obj.Pmat);
        end
        
        function [err] = angularError(obj, other)
            p1 = normc(obj.toVec());
            p2 = normc(other.toVec());
            err = acosd(min(abs(p1'*p2), 1));
        end
        
        % name needs to be fixed but we will fix it anyway in the
        % presentation
        function [err] = projectError(obj, pts2d, pts3d)
            pts2dProj = pts3d.project(obj);
            err = pts2dProj.SqError(pts2d);
        end
        
        function [err] = projectErrorAng(obj, pts2d, pts3d)
            pts2dProj = pts3d.project(obj);
            err = pts2dProj.angularError(pts2d);
        end
        
        % use only if P is expected to be euclidean
        function [posesOut] = toPoses(obj)
            P = obj.Pmat;
            s = (norm(P(:, 1)) * norm(P(:, 2)) * norm(P(:, 3)))^(1 / 3);
            P = P / s;
            
            t = P(:, 4);
            
            [U, ~, V] = svd(P(1:3, 1:3));
            R = U * V';
            if det(R) < 0
                R = -1 * R;
                t = -t;
            end
            % only the first one should be the correct one..
            posesOut= {Pose([R, t]), Pose([R, -t])};
        end
        
    end
    
    methods(Static)
        function [CamsOut] = transformCams(cams, calibrations)
            nCams = length(cams);
            assert(nCams == length(calibrations));
            
            CamsOut = cell(1, nCams);
            for i = 1:nCams
                CamsOut{i} = cams{i}.transform(calibrations{i});
            end
        end
        function [Pvec] = Pmat2vec(Pmat)
            Pvec = normc(reshape(Pmat', [12, 1]));
        end
        
        function [Pmat] = Pvec2mat(Pvec)
            Pmat = reshape(Pvec, [4, 3])';
        end
    end
end
