classdef HomoMat
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Hmat
    end
    
    methods
        function obj = HomoMat(Hmat)
            obj.Hmat = Hmat;
        end
        
        % x2 = H * x1. suppose xi' = Ni * xi
        % find Hout such that x2' = Hout * x1'
        function Hout = transform(obj, N1, N2)
            Hout = HomoMat(N2*obj.Hmat/N1);
        end
        
        function Hout = invTransform(obj, N1, N2)
            Hout = HomoMat(N2 \ obj.Hmat*N1);
        end
        
        function Hout = inverse(obj)
            Hout = HomoMat(inv(obj.Hmat));
        end
        
        function Hout = normalize(obj)
            Hout = HomoMat(obj.Hmat./norm(obj.Hmat, 'fro'));
        end
        
        function pointsOut = transfer(pointsIn)
            error('not implemented')
        end
        
        function [err] = transferError(obj, points)
            x1 = points{1}.points;
            x2 = points{2}.points;
            
            points_t21 = obj.Hmat \ x2;
            points_t21 = points_t21 ./ points_t21(3, :);
            err_t21 = points_t21 - x1;
            
            
            points_t12 = obj.Hmat * x1;
            points_t12 = points_t12 ./ points_t12(3, :);
            err_t12 = points_t12 - x2;
            
            err = 0.5 * (sum(err_t21.^2 + err_t12.^2, 1));
        end
        
        function [err] = transferError_stable(obj, points)
            x1 = points{1}.points;
            x2 = points{2}.points;
            
            points_t21 = obj.Hmat \ x2;
            points_t21 = points_t21 ./ (points_t21(3, :)+1e-10);
            err_t21 = points_t21 - x1;
            
            
            points_t12 = obj.Hmat * x1;
            points_t12 = points_t12 ./ (points_t12(3, :)+1e-10);
            err_t12 = points_t12 - x2;
            
            err = 0.5 * (sum(err_t21.^2 + err_t12.^2, 1));
        end
        
        function [err] = angularError(obj, other)
            h1 = normc(reshape(obj.Hmat, 9, 1));
            h2 = normc(reshape(other.Hmat, 9, 1));
            err = acosd(abs(h1'*h2));
        end
        
        function posesOut = toPoses(obj)
            [R12s, T12s] = decomposeHomographyMatrix(obj.Hmat);
            nSol = size(T12s, 1);
            posesOut = cell(1, nSol);
            for i = 1:nSol
                R = R12s(:, :, i);
                t = T12s(i, :)';
                posesOut{i} = Pose([R, t]);
            end
        end
        
    end
    
    methods(Static)
       function Hout = fromSubspace(B)
            % B is the basis for space of (unfolded) fundamental matrices
            % Q can be viewed as the basis for "data" or "embeddings"
           [Q, R] = qr(B);
            Q = Q(:, 4:end);
            
            E1 = kron(eye(3),crossM([1; 0; 0]));
            E2 = kron(eye(3),crossM([0; 1; 0]));
            E3 = kron(eye(3),crossM([0; 0; 1]));
            E = [E1; E2; E3];
            A = kron(eye(3), Q');
            [U,S,V]=svd(E); Up=U(:,1:rank(E)); Vp=V(:,1:rank(E)); Sp=S(1:rank(E),1:rank(E));
            [~,~,V]=svd(A*Up); tp=V(:,end);
            t=Up*tp;
            h=Vp*inv(Sp)*tp;
            
            Hout = HomoMat(reshape(h,3,3));
        end
    end
end
