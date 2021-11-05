classdef EssenMat < FundMat
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    methods
        function obj = EssenMat(method, arg2)
%             obj.Fmat = Fmat;
            switch method
                case 'relPose'
                    E = EssenMat.fromPose(arg2);
                case 'absPose'
                    assert(length(arg2) == 2);
                    [~, posesOut] = Pose.normalizePoses(arg2);
                    E = EssenMat.fromPose(posesOut{2});                    
                case 'E'
                    E = arg2;
                    assert(all(size(E) == [3, 3]));
                    s = svd(E);
                    assert(s(1) > 1e-5);
                    assert(1-s(2)/s(1) < 1e-5);
                    assert(s(3) < 1e-5);                    
            end
            obj@FundMat(E);
        end
    end
    
    methods (Static)
         function Eout = fromPose(pose)
             R = pose.Pmat(:, 1:3);
             t = pose.Pmat(:, 4);
             Eout = crossM(t) * R;
         end
    end
end

