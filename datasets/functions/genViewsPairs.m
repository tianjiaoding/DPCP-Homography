function [datasets, views] = genViewsPairs(collection)
datasets = {};
views = {};

if strcmp(collection, 'Oxford')
    basePath = './datasets/Oxford/';
    [datasetArray] = getScenes(basePath);
    
    for i = 1:length(datasetArray)
        dataset = datasetArray{i};
        dataPath = [basePath, dataset];
        frames = getOxfordFrames(dataPath);
        [viewsLocal] = getViewsPairs(frames);
        views = [views, viewsLocal];
        datasets = [datasets, repmat({dataset}, 1, length(viewsLocal))];
    end
elseif strcmp(collection, 'EPFL')
    basePath = './datasets/EPFL/';
    [datasetArray] = getScenes(basePath);
    
    for i = 1:length(datasetArray)
        dataset = datasetArray{i};
        dataPath = [basePath, dataset];
        frames = getEPFLFrames(dataPath);
        [viewsLocal] = getViewsPairs(frames);
        views = [views, viewsLocal];
        datasets = [datasets, repmat({dataset}, 1, length(viewsLocal))];
    end
elseif strcmp(collection,'TUMRGBD')
    basePath = './datasets/TUMRGBD/';
    [datasetArray] = getScenes(basePath);
    for i = 1:length(datasetArray)
        dataset = datasetArray{i};
        dataPath = [basePath, dataset];
        frames = getTUMRGBDFrames(dataPath);
        [viewsLocal] = getViewsPairsgap(frames,20);
        views = [views, viewsLocal];
        datasets = [datasets, repmat({dataset}, 1, length(viewsLocal))];
    end
else
    error('wrong collection')
end
end

function [datasets] = getScenes(basePath)
datasets = {};
listing = dir(basePath);
numResults = size(listing, 1);
for i = 1:numResults
    if listing(i).isdir ~= 1
        continue;
    end
    if strcmp(listing(i).name, '.') || strcmp(listing(i).name, '..')
        continue;
    end
    datasets{end+1} = [listing(i).name];
end
end

function [frames] = getTUMRGBDFrames(dataPath)
load([dataPath,'/gt_camera.mat']);
frames = imgt;
end

function [frames] = getOxfordFrames(dataPath)
frames = [];
listing = dir(dataPath);
numResults = size(listing, 1);
for i = 1:numResults
    filename = listing(i).name;
    if length(filename) ~= 5
        continue;
    end
    if any(filename(4:5) ~= '.P')
        continue;
    end
    frames(end+1) = str2num(filename(1:3));
end
end

function [frames] = getEPFLFrames(dataPath)
frames = [];
listing = dir(dataPath);
numResults = size(listing, 1);

if exist([dataPath, '/0000.png'], 'file')
    for i = 1:numResults
        filename = listing(i).name;
        if length(filename) ~= 8
            continue;
        end
        if ~strcmp(filename(5:8), '.png')
            continue;
        end
        frames(end+1) = str2num(filename(1:4));
    end
elseif exist([dataPath, '/rdimage.000.ppm'], 'file')
    for i = 1:numResults
        filename = listing(i).name;
        if length(filename) ~= 15
            continue;
        end
        if ~strcmp(filename(12:15), '.ppm')
            continue;
        end
        frames(end+1) = str2num(filename(9:11));
    end
else
    error('wrong EPFL datapath')
end

end

function [views] = getViewsPairs(frames)
views = {};
for i = 1:length(frames) - 1
    views{end+1} = [frames(i), frames(i+1)];
end
end

function [views] = getViewsPairsgap(frames, gap)
views = {};
for i = 1:gap:length(frames) - gap
    views{end+1} = [frames(i), frames(i+gap)];
end
end