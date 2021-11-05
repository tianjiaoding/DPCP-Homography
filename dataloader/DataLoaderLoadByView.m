classdef (Abstract) DataLoaderLoadByView < DataLoader
    
    methods (Abstract)
        loadView(obj)
    end
    
    methods (Access = public)
        function [viewStruct] = loadViewPair(obj, scene, viewPair)
            assert(size(viewPair, 2) == 2);
            [viewStruct] = obj.loadViewSet(scene, viewPair);
        end
        
        function [viewStruct] = loadViewTriplet(obj, scene, viewTriplet)
            assert(size(viewTriplet, 2) == 3);
            [viewStruct] = obj.loadViewSet(scene, viewTriplet);
        end
    end
    
    methods (Access = protected)
        function [viewStructOut] = loadViewSet(obj, scene, viewSet)
            nViews = size(viewSet, 2);

            for i = 1:nViews
                [viewStructs(i)] = obj.loadView(scene, viewSet(1, i));
            end
            
            viewStructOut = struct();
            fieldNames = fieldnames(viewStructs);
            for j = 1:length(fieldNames)
                fieldName = fieldNames{j};
                
                % may switch-case the field names to specialize for
                % different data types
                switch fieldName
                    % FIXME: are the below two lines still used? 
                    case {'F', 'H'}
                        viewStructOut.(fieldName) = viewStructs(1).(fieldName);
                    otherwise
                        viewStructOut.([fieldName, 's']) = {viewStructs.(fieldName)};
                end
            end
        end
    end
end

