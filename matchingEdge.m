function [corespondencesEdge, corespondencesWeights] = matchingEdge(centeredPoints_1,...
    centeredPoints_2, barycenterMap_1, barycenterMap_2, barycenterThreshold)
% matching the edges and creating the weight array

corespondencesEdge = [];
corespondencesDeltaSize = [];
corespondencesMahal = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(centeredPoints_1)
    dist = 500;
    idx = 0;
    for j=1:length(centeredPoints_2)
        try
            mahaldist = mean(mahal(centeredPoints_1{i}, centeredPoints_2{j}));
            barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
            deltaSize = abs(length(centeredPoints_1{i})-length(centeredPoints_2{j}))/...
                max(length(centeredPoints_1{i}),length(centeredPoints_2{j}));
            if mahaldist < dist && barycenterDist < barycenterThreshold
                dist = mahaldist;
                optiDelta = deltaSize;
                idx = j;
            end
        catch
            warning('dimension problem');
        end
    end
    if idx~=0 && ~ismember(idx, idxList) && mahaldist ~= 0
        idxList = [idxList, idx];
        corespondencesEdge = [corespondencesEdge; [i, idx]];
        corespondencesDeltaSize = [corespondencesDeltaSize; optiDelta];
        corespondencesMahal = [corespondencesMahal; dist];
    end
end

% building the correspondences weights
corespondencesWeights = [];
weightList = corespondencesDeltaSize;
minWeight = min(weightList);
for k =1:length(corespondencesMahal)
    weight = (1+minWeight)/(weightList(k)+1);
    corespondencesWeights = [corespondencesWeights; weight];
end
end

