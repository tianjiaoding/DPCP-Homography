classdef (Abstract) DataLoader
    %DataLoader abstract dataloader class
    
    properties(Abstract)
        basePath
        matcherParam
    end
    
    methods(Abstract)
        listScenes(obj)
        listViewsByScene(obj)
        
    end
    
    methods(Access = public)
        function [scenes, viewPairs] = listViewPairs(obj, gap)
            scenes = {};
            viewPairs = {};
            
            [sceneArray] = obj.listScenes();
            for i = 1:length(sceneArray)
                scene = sceneArray{i};
                frames = obj.listViewsByScene(scene);
                [viewPairsLocal] = DataLoader.getViewsPairsgap(frames, gap);
                viewPairs = [viewPairs, viewPairsLocal];
                scenes = [scenes, repmat({scene}, 1, length(viewPairsLocal))];
            end
        end
        
        function [scenes, viewTriplets] = listViewTriplets(obj, gap)
            scenes = {};
            viewTriplets = {};
            
            [sceneArray] = obj.listScenes();
            for i = 1:length(sceneArray)
                scene = sceneArray{i};
                frames = obj.listViewsByScene(scene);
                [viewTripletsLocal] = DataLoader.getViewsTripletsgap(frames, gap);
                viewTriplets = [viewTriplets, viewTripletsLocal];
                scenes = [scenes, repmat({scene}, 1, length(viewTripletsLocal))];
            end
        end
    end
    
    
    methods(Static, Access = protected)
        function [namesOut] = listDirNames(basePath)
            namesOut = {};
            listing = dir(basePath);
            numResults = size(listing, 1);
            for i = 1:numResults
                if listing(i).isdir ~= 1
                    continue;
                end
                if strcmp(listing(i).name, '.') || strcmp(listing(i).name, '..')
                    continue;
                end
                namesOut{end+1} = [listing(i).name];
            end
        end
        
        % FIXME: is there a more reasonable way of traversing the frames?
        function [views] = getViewsPairsgap(frames, gap)
            views = {};
            for i = 1:gap:length(frames) - gap
                views{end+1} = [frames(i), frames(i+gap); ...
                    i, i + gap]; % for indexing image files
            end
        end
        
        % FIXME: is there a more reasonable way of traversing the frames?
        function [views] = getViewsTripletsgap(frames, gap)
            views = {};
            for i = 1:gap:length(frames) - 2 * gap
                views{end+1} = [frames(i), frames(i+gap), frames(i+2*gap); ...
                    i, i + gap, i + 2 * gap]; % for indexing image files
            end
        end
        
    end
end
