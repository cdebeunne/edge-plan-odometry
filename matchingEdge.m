function [corespondencesEdge, corespondencesWeights] = matchingEdge(centeredPoints_1,...
    centeredPoints_2, barycenterMap_1, barycenterMap_2, barycenterThreshold)
% matching the edges and creating the weight array

corespondencesEdge = [];
corespondencesDeltaSize = [];
corespondencesMahal = [];
svdList = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(centeredPoints_1)
    dist = 100;
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
        corespondencesMahal = [corespondencesMahal; dist/100];
        svdList = [svdList, svd(cov(centeredPoints_1{i}))];
    end
end

% building the correspondences weights
corespondencesWeights = ones(length(corespondencesDeltaSize),1);
svdWeight = svdList(2,:)./svdList(1,:);
weightList = svdWeight;
minWeight = min(weightList);
for k =1:length(corespondencesDeltaSize)
    weight = (1+minWeight)/(weightList(k)+1);
    weight = weight^3;
    corespondencesWeights(k) = weight;
end
end

