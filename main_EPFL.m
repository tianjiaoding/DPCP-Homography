clear all;
close all;

dataLoader = DataLoaderEPFL;

% frame gap could be changed here
[scenes, viewPairs] = dataLoader.listViewPairs(1);
for idViewPair = 1:length(viewPairs)
    scene = scenes{idViewPair};
    viewPair = viewPairs{idViewPair};
    
    % load images, calibration, and ground-truth poses
    [VS] = dataLoader.loadViewPair(scene, viewPair);
    
    
    % feature matching
    featMatcher = FeatureMatcher(dataLoader.matcherParam);
    [matchedPoints, metrics] = featMatcher.match2views(VS.images);
    
    disp(scene)
            
    % visualize correspondences and their reproj errors w.r.t. gt poses
    VS.cameras = Camera.transformCams(VS.poses, VS.calibrations);
    reprojErrF = TwoViewGeometry.reprojError(VS.cameras, matchedPoints);
    
%     figure;
%     stem(reprojErrF);
%     ylim([0, 10])
%     TwoViewGeometry.visCorrespWithScore(VS.images, matchedPoints, reprojErrF, ...
%         [0, 10]);

    problem = TwoViewProblem(matchedPoints, VS.images, ...
        'poses', VS.poses, 'calibrations', VS.calibrations, 'computeF', false);

    % fundamental matrix estimation
    % data: epipolar embeddings of correspondences
    % method: DPCP learning a subspace of codimension 3
    algo = AlgoDPCP;
    result = problem.goFund(algo);
    result
    
    % homography estimation
    % data: homographic embeddings of correspondences
    % method: DPCP learning a subspace of codimension 1
    algo = AlgoDPCP;
    result = problem.goHomoC1(algo);
    result
    
    % homography estimation
    % data: epipolar embeddings of correspondences
    % method: DPCP learning a subspace of codimension 3
    algo = AlgoDPCP;
    result = problem.goHomoC3(algo);
    result
end