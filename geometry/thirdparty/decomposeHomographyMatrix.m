% Compute the 4 possible pairs of R and t from homography matrix

% Copyright 2018 The MathWorks, Inc.

% References:
% -----------
% [1] O. Faugeras and F. Lustman, “Motion and structure from motion in a
% piecewise planar environment”, in International Journal of Pattern
% Recognition and Artificial Intelligence, 2(3):485–508, 1988.

%#codegen

function [Rs, Ts, Ns] = decomposeHomographyMatrix(A)

[U, S, V] = svd(A);
% Eq. 8
s = det(U)*det(V);
% singular values are nonnegative
d1 = S(1, 1);
d2 = S(2, 2);
d3 = S(3, 3);

outputClass = class(A);
cond1 = abs(d1-d2)<10^3*eps(outputClass); % d1 == d2
cond2 = abs(d2-d3)<10^3*eps(outputClass); % d2 == d3
if (cond1 && ~cond2) % translation is normal to the plane
    [Rs, Ts, Ns] = computeTwoSingularCaseA(d1, d2, d3, U, V, s);
    
elseif (~cond1 && cond2)
    [Rs, Ts, Ns] = computeTwoSingularCaseB(d1, d2, d3, U, V, s);
    
elseif (cond1 && cond2) % pure rotation
    Ts = zeros(1, 3, outputClass);
    Ns = nan(1, 3); % normal is undefined

    Rp = eye(3, outputClass);
    Rs = s*U*Rp*V';    
    % Another degenerate case is a transparent plane is observed from the
    % two opposite sides and at the same distance. The normal is still
    % undefined, and this is not considered here.
else
    [Rs, Ts, Ns] = computeGeneralCase(d1, d2, d3, U, V, s);
end

function [Rs, Ts, Ns] = computeTwoSingularCaseA(d1, ~, d3, U, V, s)

outputClass = class(d1);

Rs = zeros(3,3,2,outputClass);
Ts = zeros(2,3,outputClass);
Ns = zeros(2,3,outputClass);

Rp = eye(3, outputClass);
R = s*U*Rp*V';

x = [1 -1];
for i = 1 : 2
    Rs(:,:,i) = R;
    np = [0,0,x(i)];
    tp = (d3-d1)*np;
    t = tp * U';
    n = np * V';
    Ts(i, :) = t/norm(t);
    Ns(i, :) = n/norm(n);
end
% in case d < 0
% Rp = eye(3, outputClass);
% Rp([1,5]) = -1;
% R = s*U*Rp*V';
% 
% x = [1 -1];
% for i = 1 : 2
%     Rs(:,:,i) = R;
%     np = [0,0,x(i)];
%     tp = (d3+d1)*np;
%     t = tp * U';
%     n = np * V';
%     Ts(i, :) = t/norm(t);
%     Ns(i, :) = n/norm(n);
% end

function [Rs, Ts, Ns] = computeTwoSingularCaseB(d1, ~, d3, U, V, s)

outputClass = class(d1);

Rs = zeros(3,3,2,outputClass);
Ts = zeros(2,3,outputClass);
Ns = zeros(2,3,outputClass);

Rp = eye(3, outputClass);
R = s*U*Rp*V';

x = [1 -1];
for i = 1 : 2
    Rs(:,:,i) = R;
    np = [x(i),0,0];
    tp = (d1-d3)*np;
    t = tp * U';
    n = np * V';
    Ts(i, :) = t/norm(t);
    Ns(i, :) = n/norm(n);
end
% in case d < 0
% Rp = eye(3, outputClass);
% Rp([1,5]) = -1;    
% R = s*U*Rp*V';
% 
% x = [1 -1];
% 
% for i = 1 : 2
%     Rs(:,:,i) = R;
%     np = [x(i),0,0];
%     tp = (d1+d3)*np;
%     t = tp * U';
%     n = np * V';
%     Ts(i, :) = t/norm(t);
%     Ns(i, :) = n/norm(n);
% end


function [Rs, Ts, Ns] = computeGeneralCase(d1, d2, d3, U, V, s)

outputClass = class(d1);

% Eq.12
aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
x1 = [aux1,aux1,-aux1,-aux1];
x2 = zeros(1, 4, outputClass);
x3 = [aux3,-aux3,aux3,-aux3];
% Eq.13
aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);
ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
stheta = [aux_stheta, -aux_stheta, -aux_stheta, aux_stheta];

Rs = zeros(3,3,4,outputClass);
Ts = zeros(4,3,outputClass);
Ns = zeros(4,3,outputClass);
for i = 1:4
    Rp = [ctheta, 0, -stheta(i); ...
           0,     1,   0; ...
          stheta(i), 0, ctheta];
    R = s*U*Rp*V';
    Rs(:,:,i) = R;

    %Eq.14
    tp = (d1-d3)*[x1(i), x2(i), -x3(i)];
    t = tp * U';
    Ts(i, :) = t/norm(t);

    np = [x1(i), x2(i), x3(i)];
    n = np * V';
    Ns(i, :) = n/norm(n);
end

% in case d < 0
% Eq. 15, 16
% aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);
% cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
% sphi = [aux_sphi, -aux_sphi, -aux_sphi, aux_sphi];
% 
% for i = 1:4
%     Rp = [cphi, 0, sphi(i); ...
%            0,   -1,   0; ...
%           sphi(i), 0, -cphi];
%     R = s*U*Rp*V';
%     Rs(:,:,i) = R;
% 
%     tp = (d1+d3)*[x1(i), x2(i), x3(i)];
%     t = tp * U';
%     Ts(i, :) = t/norm(t);
% 
%     np = [x1(i), x2(i), x3(i)];
%     n = np * V';
%     Ns(i, :) = n/norm(n);
% end
