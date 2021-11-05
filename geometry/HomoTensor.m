classdef HomoTensor
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Htsr
    end
    
    methods
        function obj = HomoTensor(Htsr)
            obj.Htsr = Htsr;
        end
        
        % x2' * F * x1 = 0. suppose xi' = Ni * xi
        % find Fout such that x2' * Fout * x1 = 0
        function Hout = transform(obj, N1, N2, N3)
            [H21, H31] = ABfromHTensor(obj.Htsr);
            H12 = inv(H21);
            H13 = inv(H31);
            H12_de = normalizeHomo(H12, N1, N2, 1);
            H13_de = normalizeHomo(H13, N1, N3, 1);
            H21_de = inv(H12_de);
            H31_de = inv(H13_de);
            Hout = HomoTensor(HTensorFromHomographies(H21_de, H31_de));
        end
        
        function Hout = invTransform(obj, N1, N2, N3)
            [H21, H31] = ABfromHTensor(obj.Htsr);
            H12 = inv(H21+10^(-10)*eye(3));
            H13 = inv(H31+10^(-10)*eye(3));
            H12_de = normalizeHomo(H12, N1, N2, 0);
            H13_de = normalizeHomo(H13, N1, N3, 0);
            H21_de = inv(H12_de+10^(-10)*eye(3));
            H31_de = inv(H13_de+10^(-10)*eye(3));
            Hout = HomoTensor(HTensorFromHomographies(H21_de, H31_de));
        end
        
        function [err] = transferError(obj, points)
            x1 = points{1}.points;
            x2 = points{2}.points;
            x3 = points{3}.points;
            [H21, H31] = ABfromHTensor(obj.Htsr);
            H12 = inv(H21);
            H13 = inv(H31);
            
            points_t21 = H12 \ x2;
            points_t21 = points_t21 ./ points_t21(3, :);
            err_t21 = points_t21 - x1;
            
            points_t12 = H12 * x1;
            points_t12 = points_t12 ./ points_t12(3, :);
            err_t12 = points_t12 - x2;
            
            points_t31 = H13 \ x3;
            points_t31 = points_t31 ./ points_t31(3, :);
            err_t31 = points_t31 - x1;
            
            points_t13 = H13 * x1;
            points_t13 = points_t13 ./ points_t13(3, :);
            err_t13 = points_t13 - x3;
            
            err = 0.25 * (sum(err_t21.^2 + err_t12.^2+err_t31.^2 + err_t13.^2, 1));
        end
        
        function [err] = transferError_stable(obj, points)
            x1 = points{1}.points;
            x2 = points{2}.points;
            x3 = points{3}.points;
            [H21, H31] = ABfromHTensor(obj.Htsr);
            H12 = inv(H21);
            H13 = inv(H31);
            
            points_t21 = H12 \ x2;
            points_t21 = points_t21 ./ (points_t21(3, :)+1e-10);
            err_t21 = points_t21 - x1;
            
            points_t12 = H12 * x1;
            points_t12 = points_t12 ./ (points_t12(3, :)+1e-10);
            err_t12 = points_t12 - x2;
            
            points_t31 = H13 \ x3;
            points_t31 = points_t31 ./ (points_t31(3, :)+1e-10);
            err_t31 = points_t31 - x1;
            
            points_t13 = H13 * x1;
            points_t13 = points_t13 ./ (points_t13(3, :)+1e-10);
            err_t13 = points_t13 - x3;
            
            err = 0.25 * (sum(err_t21.^2 + err_t12.^2+err_t31.^2 + err_t13.^2, 1));
        end
        
        function [err] = angularError(obj, other)
            h1 = normc(reshape(obj.Htsr, 27, 1));
            h2 = normc(reshape(other.Htsr, 27, 1));
            err = acosd(min(abs(h1'*h2), 1));
        end
        
        function [t] = toUnitVec(obj)
            t = normc(reshape(obj.Htsr, 27, 1));
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
