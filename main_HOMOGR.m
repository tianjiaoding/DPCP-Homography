clear all;
close all;

dataLoader = DataLoaderHOMOGR;


[scenes, viewPairs] = dataLoader.listViewPairs(1);
idPairsWithSufficientInliers = [2, 5, 13, 15, 16];
% for idViewPair = 1:length(viewPairs)
for idViewPair = idPairsWithSufficientInliers
    scene = scenes{idViewPair};
    viewPair = viewPairs{idViewPair};
    
    % load images, calibration, and ground-truth poses
    [VS] = dataLoader.loadViewPair(scene, viewPair);
    
    % feature matching
    featMatcher = FeatureMatcher(dataLoader.matcherParam);
    [matchedPoints, metrics] = featMatcher.match2views(VS.images);
    
    disp(scene)
%     VS.H.Hmat
    
    % visualize correspondences and their transfer errors w.r.t. H_gt
    transferErrH = VS.H.transferError(matchedPoints);

%     TwoViewGeometry.visCorrespWithScore(VS.images, matchedPoints, transferErrH, ...
%         [0, 10]);
    
    problem = TwoViewProblem(matchedPoints, VS.images, ...
            'valPoints', VS.valCorresp, 'H', VS.H);
    
    disp(num2str(idViewPair))
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