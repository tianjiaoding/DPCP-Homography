classdef FundMat
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Fmat
    end
    
    methods
        function obj = FundMat(Fmat)
            obj.Fmat = Fmat;
        end
        
        % x2' * F * x1 = 0. suppose xi' = Ni * xi
        % find Fout such that x2' * Fout * x1 = 0
        function Fout = transform(obj, N1, N2)
            Fout = FundMat(N2' \ obj.Fmat / N1);
        end
        
        function Fout = invTransform(obj, N1, N2)
            Fout = FundMat(N2' * obj.Fmat * N1);
        end
        
        function [err] = sampsonError(obj, points)
            x1 = points{1}.points;
            x2 = points{2}.points;

            err = zeros(1, size(x1, 2));

            Fx1 = obj.Fmat*x1;
            epipolarDists = sum(x2 .* Fx1);
            Ftx2 = obj.Fmat'*x2;

            % Sampson distance
            err =  0.5 * epipolarDists.^2 ./ (sum(Fx1(1:2, :).^2) +  sum(Ftx2(1:2,:).^2));
            
%             err =  0.5 * epipolarDists.^2 ./ (sum(Fx1(1:2, :).^2) +  sum(Ftx2(1:2,:).^2));
%             err = err.^0.5;
        end
        
        function [err] = symmetricEpipolarDistance(obj, points)
            x1 = points{1}.points;
            x2 = points{2}.points;

            err = zeros(1, size(x1, 2));
            Ftx2 = obj.Fmat'*x2;
            epipolarDists = sum(x1 .* Ftx2);
            Fx1 = obj.Fmat*x1;
            a = Ftx2(1,:).^2+Ftx2(2,:).^2;
            b = Fx1(1,:).^2 + Fx1(2,:).^2;
            
            err = epipolarDists.^2.*(a+b)./(a.*b);
%             err = err.^0.5;
        end
        
        function [err] = angularError(obj, other)
            f1 = normc(reshape(obj.Fmat, 9, 1));
            f2 = normc(reshape(other.Fmat, 9, 1));
            err = acosd(min(abs(f1'*f2), 1));
        end
        
        function [f] = toUnitVec(obj)
            f = normc(reshape(obj.Fmat, 9, 1));
        end
        
        function Fout = toValid(obj)
            [U,S,V] = svd(obj.Fmat);
            S(end,end) = 0;
            Fout = FundMat(U*S*V');
        end
        
        function posesOut = toPoses(obj)
            W = [0, -1, 0; 1, 0, 0; 0, 0, 1];
            [U, ~, V] = svd(obj.Fmat);
            R = U * W * V.';
            Rp = U * W.' * V.';
            R = R * sign(det(R));
            Rp = Rp * sign(det(Rp));
            t = U(:, 3);
            
            posesOut = cell(1, 4);
            for k = 1:4
                if k == 2 || k == 4
                    t = -t;
                elseif k == 3
                    R = Rp;
                end
                posesOut{k} = Pose([R, t]);
            end
        end
    end
end
