classdef AlgoDPCP < AlgoRPCA
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        params = struct('c', 1, 'delta', 1e-9, 'epsilon_J', 1e-6, ...
            'iter', 1000, 'g', 1);
    end
    
    methods
        function algoOut = run(obj, data)
            [~, B, ~, time, ~, iter] = DPCP_IRLS_G_2tb...
                (data, ...
                obj.params.c, ...
                obj.params.delta,...
                obj.params.iter,...
                obj.params.epsilon_J, ...
                obj.params.g, ...
                inf, inf);
            disp(['DPCP-IRLS terminates with ', num2str(iter), ' iterations.']);
            algoOut = struct();
            algoOut.B = B;
            algoOut.time = time;
            algoOut.iter = iter;
            algoOut.LOiter = 0;
            algoOut.Obj = sum(abs(normc(data)'*B));
        end
    end
end

