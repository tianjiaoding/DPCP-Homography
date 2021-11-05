function [Matcher3view, Matcher2view12, Matcher2view13, timeMatching] = ...
    matchAndClassify_2v(images, cameras, th_coeff, matchthres, maxratio, feattype, calibrations)

th = sqrt(3) * th_coeff * abs(calibrations{1}(1, 1));
th_2v = sqrt(2) * th_coeff * abs(calibrations{1}(1, 1));

% th = sqrt(3);
% th_2v = sqrt(2);
% warning('th=sqrt(3) and th_2v=sqrt(2)')

if nargin <= 3
    matchthres = 50;
    maxratio = 0.85;
end

if nargin <= 5
    feattype = 'ORB';
end

% if nargin <=6
rois = [];
% end

I1 = images{1};
I2 = images{2};
I3 = images{3};

t_detection = 0;
t_extraction = 0;
t_matching = 0;

%% Feature detection
t_start = tic;
switch feattype
    case 'ORB'
        disp('Detecting ORB features');
        if ~isempty(rois)
            points1 = detectORBFeatures(I1, 'ROI', rois{1}.Position);
            points2 = detectORBFeatures(I2, 'ROI', rois{2}.Position);
            points3 = detectORBFeatures(I3, 'ROI', rois{3}.Position);
        else
            points1 = detectORBFeatures(I1);
            points2 = detectORBFeatures(I2);
            points3 = detectORBFeatures(I3);
        end
    case 'FAST'
        disp('Detecting FAST features');
        points1 = detectFASTFeatures(I1, 'MinQuality', 0.1, 'MinContrast', 0.1);
        points2 = detectFASTFeatures(I2, 'MinQuality', 0.1, 'MinContrast', 0.1);
        points3 = detectFASTFeatures(I3, 'MinQuality', 0.1, 'MinContrast', 0.1);
    case 'Harris'
        disp('Detecting Harris features');
        points1 = detectHarrisFeatures(I1);
        points2 = detectHarrisFeatures(I2);
        points3 = detectHarrisFeatures(I3);
    case 'BRISK'
        disp('Detecting BRISK features');
        points1 = detectBRISKFeatures(I1, 'NumOctaves', 1);
        points2 = detectBRISKFeatures(I2, 'NumOctaves', 1);
        points3 = detectBRISKFeatures(I3, 'NumOctaves', 1);
    case 'SURF'
        disp('Detecting SURF features');
        points1 = detectSURFFeatures(I1, 'MetricThreshold', 500);
        points2 = detectSURFFeatures(I2, 'MetricThreshold', 500);
        points3 = detectSURFFeatures(I3, 'MetricThreshold', 500);
end
t_detection = toc(t_start);
disp(['time for feature detection: ', num2str(t_detection)]);

%% Feature extraction
t_start = tic;
switch feattype
    case 'ORB'
        disp('Extracting ORB features');
        [features1, valid_points1] = extractFeatures(I1, points1, 'Method', 'ORB');
        [features2, valid_points2] = extractFeatures(I2, points2, 'Method', 'ORB');
        [features3, valid_points3] = extractFeatures(I3, points3, 'Method', 'ORB');
    case 'BRISK'
        disp('Extracting BRISK features');
        [features1, valid_points1] = extractFeatures(I1, points1, 'Method', 'BRISK');
        [features2, valid_points2] = extractFeatures(I2, points2, 'Method', 'BRISK');
        [features3, valid_points3] = extractFeatures(I3, points3, 'Method', 'BRISK');
    case 'SURF'
        disp('Extracting SURF features');
        [features1, valid_points1] = extractFeatures(I1, points1, 'Method', 'SURF', 'Upright', true);
        [features2, valid_points2] = extractFeatures(I2, points2, 'Method', 'SURF', 'Upright', true);
        [features3, valid_points3] = extractFeatures(I3, points3, 'Method', 'SURF', 'Upright', true);
    case 'FAST'
        disp('Extracting FAST features');
        [features1, valid_points1] = extractFeatures(I1, points1);
        [features2, valid_points2] = extractFeatures(I2, points2);
        [features3, valid_points3] = extractFeatures(I3, points3);
    otherwise
        disp('Extracting other features');
        error('not implemented');
end
t_extraction = toc(t_start);
disp(['time for feature extraction: ', num2str(t_extraction)]);

%% Feature matching between each 2 views
t_start = tic;
if ~isempty(rois)% not used !! to be decided
    [idPairs12, metrics12] = matchFeatures(features1, features2, 'MatchThreshold', 95, 'MaxRatio', 0.99);
    [idPairs13, metrics13] = matchFeatures(features1, features3, 'MatchThreshold', 95, 'MaxRatio', 0.99);
else
    [idPairs12, metrics12] = matchFeatures(features1, features2, 'MatchThreshold', matchthres, 'MaxRatio', maxratio, 'Unique', true);
    [idPairs13, metrics13] = matchFeatures(features1, features3, 'MatchThreshold', matchthres, 'MaxRatio', maxratio, 'Unique', true);
%     [idPairs12, metrics12] = matchFeatures(features1, features2, 'Method', 'Approximate', 'MatchThreshold', matchthres, 'MaxRatio', maxratio, 'Unique', false);
%     [idPairs13, metrics13] = matchFeatures(features1, features3, 'Method', 'Approximate', 'MatchThreshold', matchthres, 'MaxRatio', maxratio, 'Unique', false);
end

%% visualization for debugging
% plotidx = [1:10:2000];
% matchedBoxPoints = valid_points1(idPairs12(plotidx, 1), :);
% matchedScenePoints = valid_points2(idPairs12(plotidx, 2), :);
% figure(1);
% showMatchedFeatures(I1, I2, matchedBoxPoints,matchedScenePoints, 'montage');

%% join matched correspondence between view12 and view13
idPairs123 = [];
metrics123 = [];
for j = 1:size(idPairs12, 1)
    id1 = idPairs12(j, 1);
    j13Array = find(idPairs13(:, 1) == id1);
    if isempty(j13Array)
        continue;
    end
    for j13 = j13Array
        idPairs123 = [idPairs123; idPairs12(j, 1), idPairs12(j, 2), idPairs13(j13, 2)];
        metrics123 = [metrics123; metrics12(j), metrics13(j13)];
    end
end

inliers = cell(1, 3);
outliers = cell(1, 3);
inliers12 = cell(1, 2);
outliers12 = cell(1, 2);
inliers13 = cell(1, 2);
outliers13 = cell(1, 2);
if isempty(idPairs123)
    disp('failed to match any features across 3 views');
    inliers = cell(1, 3);
    outliers = cell(1, 3);
    points3D = [];
    timeMatching = 0;
    inliers_metrics = [];
    outliers_metrics = [];
    inliers_metrics12 = [];outliers_metrics12 = [];
    inliers_metrics13 = [];outliers_metrics13 = [];
    inliers_metrics = [];outliers_metrics = [];
    return;
end
%% load the location of the matched features
% across 3views
matchedPoints1 = valid_points1.Location(idPairs123(:, 1), :);
matchedPoints2 = valid_points2.Location(idPairs123(:, 2), :);
matchedPoints3 = valid_points3.Location(idPairs123(:, 3), :);
% across 2 views
matchedPoints12_1 = valid_points1.Location(idPairs12(:, 1), :);
matchedPoints12_2 = valid_points2.Location(idPairs12(:, 2), :);
matchedPoints13_1 = valid_points1.Location(idPairs13(:, 1), :);
matchedPoints13_3 = valid_points3.Location(idPairs13(:, 2), :);
t_matching = toc(t_start);
disp(['time for feature matching: ', num2str(t_matching)]);

timeMatching = t_detection + t_extraction + t_matching;

%% Classify points based on reprojection error using ground-truth cameras
%% find the 3view inliers
points = {matchedPoints1', matchedPoints2', matchedPoints3'};
n_points = size(matchedPoints1, 1);
disp([num2str(n_points), ' matched points across 3 views']);
Corresp = [matchedPoints1'; matchedPoints2'; matchedPoints3'];
Reconst0 = triangulation3D(cameras, Corresp);
Reconst0 = bsxfun(@rdivide, Reconst0(1:3, :), Reconst0(4, :));
Corresp_new = project3Dpoints(Reconst0, cameras);
residuals = Corresp_new - Corresp;
scores = vecnorm(residuals, 2, 1);
[scores_sorted, indices_sorted] = sort(scores);
id_inliers = find(scores_sorted <= th);
id_outliers = setdiff(1:n_points, id_inliers);
for i = 1:3
    inliers{i} = double(points{i}(:, indices_sorted(id_inliers)));
    inliers{i}(3, :) = 1;
    outliers{i} = double(points{i}(:, indices_sorted(id_outliers)));
    outliers{i}(3, :) = 1;
end
inliers_metrics = metrics123(indices_sorted(id_inliers), :);
outliers_metrics = metrics123(indices_sorted(id_outliers), :);
points3D = Reconst0(:, indices_sorted(id_inliers));
Matcher3view = struct();
Matcher3view.inliers = inliers;
Matcher3view.outliers = outliers;
Matcher3view.inlier_matrics = inliers_metrics;
Matcher3view.outliers_metrics = outliers_metrics;
Matcher3view.points3D = points3D;

%% find the 2view inliers between view1&2
points12 = {matchedPoints12_1', matchedPoints12_2'};
n_points12 = size(matchedPoints12_1, 1);
disp([num2str(n_points12), ' matched points across views 1,2']);
Corresp_12 = [matchedPoints12_1'; matchedPoints12_2'];
Reconst_12 = triangulation3D({cameras{1:2}}, Corresp_12);
Reconst_12 = Reconst_12(1:3, :) ./ Reconst_12(4, :);
Corresp_new12 = project3Dpoints(Reconst_12, {cameras{1:2}});
score12 = vecnorm(Corresp_new12-Corresp_12, 2, 1);
disp(num2str([mean(score12), median(score12)]));
[scores_sorted, indices_sorted] = sort(score12);
id_inliers = find(scores_sorted <= th_2v);
id_outliers = setdiff(1:size(Corresp_12, 2), id_inliers);
for i = 1:2
    inliers12{i} = double(points12{i}(:, indices_sorted(id_inliers)));
    inliers12{i}(3, :) = 1;
    outliers12{i} = double(points12{i}(:, indices_sorted(id_outliers)));
    outliers12{i}(3, :) = 1;
end
inliers_metrics12 = metrics12(indices_sorted(id_inliers), :);
outliers_metrics12 = metrics12(indices_sorted(id_outliers), :);
points3D12 = Reconst_12(:, indices_sorted(id_inliers));
Matcher2view12 = struct();
Matcher2view12.inliers = inliers12;
Matcher2view12.outliers = outliers12;
Matcher2view12.inlier_matrics = inliers_metrics12;
Matcher2view12.outliers_metrics = outliers_metrics12;
Matcher2view12.points3D = points3D12;

%% find the 2view inliers between view1&3
points13 = {matchedPoints13_1', matchedPoints13_3'};
n_points13 = size(matchedPoints13_1, 1);
disp([num2str(n_points13), ' matched points across views 1,3']);
Corresp_13 = [matchedPoints13_1'; matchedPoints13_3'];
Reconst_13 = triangulation3D({cameras{1}, cameras{3}}, Corresp_13);
Reconst_13 = Reconst_13(1:3, :) ./ Reconst_13(4, :);
Corresp_new13 = project3Dpoints(Reconst_13, {cameras{1}, cameras{3}});
score13 = vecnorm(Corresp_new13-Corresp_13, 2, 1);
disp(num2str([mean(score13), median(score13)]));
[scores_sorted, indices_sorted] = sort(score13);
id_inliers = find(scores_sorted <= th_2v);
id_outliers = setdiff(1:size(Corresp_13, 2), id_inliers);
for i = 1:2
    inliers13{i} = double(points13{i}(:, indices_sorted(id_inliers)));
    inliers13{i}(3, :) = 1;
    outliers13{i} = double(points13{i}(:, indices_sorted(id_outliers)));
    outliers13{i}(3, :) = 1;
end
inliers_metrics13 = metrics13(indices_sorted(id_inliers), :);
outliers_metrics13 = metrics13(indices_sorted(id_outliers), :);
points3D13 = Reconst_13(:, indices_sorted(id_inliers));
Matcher2view13 = struct();
Matcher2view13.inliers = inliers13;
Matcher2view13.outliers = outliers13;
Matcher2view13.inlier_matrics = inliers_metrics13;
Matcher2view13.outliers_metrics = outliers_metrics13;
Matcher2view13.points3D = points3D13;

%% perform Hartley normalization based on 3view correspondence
% newpoints = cell(1, 3);
% normalizationMat = cell(1, 3);
% for i = 1:3
%     newpoints{i} = points{i};
%     newpoints{i}(3, :) = 1;
%     assert(norm(newpoints{i}(3, :)-1) == 0);
%     [newpoints{i}(1:2, :), normalizationMat{i}] = Normalize2Ddata(newpoints{i}(1:2, :));
% end
% T_gt = vgg_T_from_P(cameras);
% T_gt = transform_TFT(T_gt, normalizationMat{1}, ...
%     normalizationMat{2}, ...
%     normalizationMat{3}, 0);
% % unfold the trifocal tensor T_gt
% t_gt = normc(reshape(T_gt, 27, 1)); 
% X_tilde = normc(tensorProdMatrix3(newpoints{1}, ...
%     newpoints{2}, newpoints{3}));

%% visualization for debugging
% [~, i_scores] = sort(scores);
% scores2 = computeAverageSubspaceDistance(t_gt, X_tilde, 4);
% figure;
% subplot(1, 2, 1);
% ecdf(scores);
% title('sqrt(d1^2+d2^2+d3^2)');
% subplot(1, 2, 2);
% stem(scores2(i_scores));
% set(gca, 'YDir', 'reverse');
% view(-90, 90);
% xlim([1, n_points]);
% title('corresponding ss avg dist');
% th = input('Please pick a threshold');
% if isempty(th)
%     disp('using default threshold 3*sqrt(3)');
%     th = 3 * sqrt(3);
% end
end