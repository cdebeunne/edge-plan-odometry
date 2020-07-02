function corespondencesEdge = matchingEdge(edgeStruct_1, edgeStruct_2,...
    detector_params)
% matching the edges and creating the weight array

%% initialization

corespondencesEdge = [];

%% edge checking

idxList = [];  
for i=1:length(edgeStruct_1.edgePoints)
    dist = 5;
    idx = 0;
    for j=1:length(edgeStruct_2.edgePoints)
        try
            eigenDist = norm(edgeStruct_1.eigen(:,i)-edgeStruct_2.eigen(:,j));
            barycenterDist = norm(edgeStruct_1.barycenterMap(i,1:2)...
                -edgeStruct_2.barycenterMap(j,1:2));
            deltaSize = abs(length(edgeStruct_1.edgePoints{i})...
                -length(edgeStruct_2.edgePoints{j}))/...
                max(length(edgeStruct_1.edgePoints{i}),...
                length(edgeStruct_2.edgePoints{j}));
            
            if eigenDist < dist &&...
                    barycenterDist < detector_params.barycenterThresholdEdge &&...
                    deltaSize<0.3
                dist = eigenDist;
                idx = j;
            end
        catch
            warning('dimension problem');
        end
    end
    if idx~=0
        idxList = [idxList, idx];
        corespondencesEdge = [corespondencesEdge; [i, idx]];
    end
end

end

