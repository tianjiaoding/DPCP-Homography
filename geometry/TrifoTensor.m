classdef TrifoTensor
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Ttsr
    end
    
    methods
        function obj = TrifoTensor(Ttsr)
            obj.Ttsr = Ttsr;
        end
        
        % x2' * F * x1 = 0. suppose xi' = Ni * xi
        % find Fout such that x2' * Fout * x1 = 0
        function Tout = transform(obj, N1, N2, N3)
            Tout = TrifoTensor(transform_TFT(obj.Ttsr, N1, N2, N3, 0));
        end
        
        function Tout = invTransform(obj, N1, N2, N3)
            Tout = TrifoTensor(transform_TFT(obj.Ttsr, N1, N2, N3, 1));
        end
        
        function [err] = sampsonError(obj, points)
            x1 = points{1}.points(1:2, :);
            x2 = points{2}.points(1:2, :);
            x3 = points{3}.points(1:2, :);
%             t = obj.Ttsr.toUnitVec();
            t = obj.toUnitVec();
            
            nPoints = size(x1, 2);
            err = zeros(1, nPoints);            
            
            % precomputing
            e6 = eye(6);
            for idp = 1:nPoints
                JJ_T = zeros(4, 4);
                
                point = [x1(:, idp); x2(:, idp); x3(:, idp)];
                A = TrifoTensor.computeA(point);
                for i = 1:6
                    y = (TrifoTensor.computeA(point+e6(:, i)) - TrifoTensor.computeA(point)) * t;
                    JJ_T = JJ_T + y * y';
                end
                JJ_T = JJ_T+1e-8*eye(4);
                lambda = JJ_T \ (-A * t);
                err(idp) = -(A * t)' * lambda;
            end
            
            err = err / 3;
        end
        
        
        function [err] = angularError(obj, other)
            t1 = normc(reshape(obj.Ttsr, 27, 1));
            t2 = normc(reshape(other.Ttsr, 27, 1));
            err = acosd(min(abs(t1'*t2), 1));
        end
        
        function [t] = toUnitVec(obj)
            t = normc(reshape(obj.Ttsr, 27, 1));
        end
        
        function [posesOut12, posesOut13] = toPoses(obj)
            % epipoles and essential matrix
            T = obj.Ttsr;
            [~,~,V]=svd(T(:,:,1)); v1=V(:,end);
            [~,~,V]=svd(T(:,:,2)); v2=V(:,end);
            [~,~,V]=svd(T(:,:,3)); v3=V(:,end);
            [~,~,V]=svd([v1 v2 v3].'); epi31=V(:,end)*sign(V(end));

            [~,~,V]=svd(T(:,:,1).'); v1=V(:,end);
            [~,~,V]=svd(T(:,:,2).'); v2=V(:,end);
            [~,~,V]=svd(T(:,:,3).'); v3=V(:,end);
            [~,~,V]=svd([v1 v2 v3].'); epi21=V(:,end)*sign(V(end));

            E21=crossM(epi21)*[T(:,:,1)*epi31 T(:,:,2)*epi31 T(:,:,3)*epi31];
            E21 = FundMat(E21);
            
            E31=-crossM(epi31)*[T(:,:,1).'*epi21 T(:,:,2).'*epi21 T(:,:,3).'*epi21];
            E31 = FundMat(E31);
            
            % TODO: figure out the relative scale of t12 and t13 from T.
            posesOut12 = E21.toPoses();
            posesOut13 = E31.toPoses();            
        end
    end
    
    methods(Static, Access = public)
        function Tout = fromCameras(cameras)
            Ps = cell(1,3);
            for i=1:3
                Ps{i} = cameras{i}.Pmat;
            end
            Tout = TrifoTensor(vgg_T_from_P(Ps));
        end
    end
    
    methods(Static, Access = private)
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
        
    end
end
