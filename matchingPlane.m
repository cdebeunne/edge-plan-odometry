function[corespondencesPlane, corespondencesWeights] = matchingPlane(centeredPoints_1,...
    centeredPoints_2, normalsPlane_1, normalsPlane_2,...
    barycenterMap_1, barycenterMap_2, barycenterThreshold)

corespondencesPlane = [];
corespondencesMahal = [];
corespondencesDeltaSize = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(centeredPoints_1)
    dist = 500;
    ratio = 0.98;
    idx = 0;
    for j=1:length(centeredPoints_2)
        try
            mahaldist = mean(mahal(centeredPoints_1{i}, centeredPoints_2{j}));
            dotRatio = dot(normalsPlane_1(:,i), normalsPlane_2(:,j))/(norm(normalsPlane_1(:,i))*norm(normalsPlane_2(:,j)));
            barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
            deltaSize = abs(length(centeredPoints_1{i})-length(centeredPoints_2{j}))/...
                max(length(centeredPoints_1{i}),length(centeredPoints_2{j}));
            if mahaldist < dist && dotRatio > ratio && barycenterDist < barycenterThreshold && deltaSize < 0.3
                dist = mahaldist;
                ratio = dotRatio;
                optiDelta = deltaSize;
                idx = j;
            end
        catch
            warning('dimension problem');
        end
    end
    if idx~=0 && ~ismember(idx, idxList) && mahaldist ~= 0
        idxList = [idxList, idx];
        corespondencesPlane = [corespondencesPlane; [i, idx]];
        corespondencesMahal = [corespondencesMahal; dist];
        corespondencesDeltaSize = [corespondencesDeltaSize;...
            optiDelta];
    end
end

% building the correspondences weights
corespondencesWeights = [];
weightList = corespondencesDeltaSize;
minWeight = min(weightList);
for k =1:length(corespondencesMahal)
    weight = (1+minWeight)/(1+weightList(k));
    corespondencesWeights = [corespondencesWeights; weight];
end
end
