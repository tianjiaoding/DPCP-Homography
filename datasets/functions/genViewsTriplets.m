function [datasets, views] = genViewsTriplets(collection)
datasets = {};
views = {};

if strcmp(collection, 'Oxford')
    basePath = './datasets/Oxford/';
    [datasetArray] = getScenes(basePath);
    
    for i = 1:length(datasetArray)
        dataset = datasetArray{i};
        dataPath = [basePath, dataset];
        frames = getOxfordFrames(dataPath);
        [viewsLocal] = getViewsTriplets(frames);
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
        [viewsLocal] = getViewsTriplets(frames);
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
        [viewsLocal] = getViewsTripletsgap(frames,20);
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

function [views] = getViewsTriplets(frames)
views = {};
for i = 1:length(frames) - 2
    views{end+1} = [frames(i), frames(i+1), frames(i+2)];
end
for i = 1:length(frames) - 4
    views{end+1} = [frames(i), frames(i+2), frames(i+4)];
end
end

% function [views] = getViewsTriplets20(frames)%be careful of using this 20 as a gap!
% views = {};
% for i = 1:20:length(frames) - 40
%     views{end+1} = [frames(i), frames(i+20), frames(i+40);i,i+20,i+40];
% end
% end

function [views] = getViewsTripletsgap(frames,gap)%be careful of using this 20 as a gap!
views = {};
for i = 1:gap:length(frames) - 2*gap
    views{end+1} = [frames(i), frames(i+gap), frames(i+2*gap);i,i+gap,i+2*gap];
end
end