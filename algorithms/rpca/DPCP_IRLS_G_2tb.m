function [normalsLowTb, normalsHighTb, timeAlgoLow, timeAlgoHigh, ...
    iterAlgoLow, iterAlgoHigh] = DPCP_IRLS_G_2tb(X_tilde, c, delta, T, epsilon_J, g, budgetLow, budgetHigh)

% solves min_B ||X^T B||_1 s.t. B^T B=I
% INPUT
% X_tilde  : DxN data matrix of N data points of dimension D
% c        : Dimension of the orthogonal complement of the subspace.
%            To fit a hyperplane use c=1.
% delta    : Avoids division by zero. Typically is set to 10^(-9).
% T        : Maximal number of iterations. Typically set to 100.
% epsilon_J: Convergence accuracy: Typically set to 10^(-6).
% OUTPUT
% distances: Distance of each point to the estimated subspace.
% B        : Dxc matrix containing in its columns an orthonormal basis for
%            the orthogonal complement of the subspace.
% budgetLow: LowTimebudget for early termination.
% budgetHigh: HighTimebudget for early termination.


% COPYRIGHT @ Manolis C. Tsakiris, 2016
savedLowTb = false;
savedHighTb = false;

t_start = tic;
[D, N] = size(X_tilde);
assert(mod(N, g) == 0);

Delta_J = Inf;
k = 0;
w = ones(g, N/g);
J_old = inf(1, N/g);
J_new = zeros(1, N/g);
while (Delta_J > epsilon_J) && (k < T)
    % the X .* w * X' way does not work for g>1
    d_w = sparse(1:N, 1:N, w);
    R_X = X_tilde * d_w * X_tilde';
    [U, ~, ~] = svd(R_X);
    B = U(:, D-c+1:D);
    J_new = vecnorm(reshape(B'*X_tilde, c*g, N/g), 2, 1);
    w = repmat(1./max(J_new, delta), g, 1);
    k = k + 1;
    Delta_J = 1 - sum(J_new) / (sum(J_old) + 10^(-9));
    J_old = J_new;
    
    t_elapsed = toc(t_start);
    if ~savedLowTb && t_elapsed > budgetLow
        normalsLowTb = B;
        timeAlgoLow = t_elapsed;
        iterAlgoLow = k;
        savedLowTb = true;
    end
    if ~savedHighTb && t_elapsed > budgetHigh
        normalsHighTb = B;
        timeAlgoHigh = t_elapsed;
        iterAlgoHigh = k;
        savedHighTb = true;
        break;
    end
end

if ~savedLowTb
    normalsLowTb = B;
    timeAlgoLow = t_elapsed;
    iterAlgoLow = k;
end

if ~savedHighTb
    normalsHighTb = B;
    timeAlgoHigh = t_elapsed;
    iterAlgoHigh = k;
end

% disp(['DPCP-IRLS-G iter: ', num2str(k), ' time: ', num2str(t_elapsed), ' t/k: ', num2str(t_elapsed/k)]);
end
