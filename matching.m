function corespondences = matching(centeredPoints_1, centeredPoints_2, barycenterMap_1, barycenterMap_2, barycenterThreshold)

corespondences = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(centeredPoints_1)
    dist = 500;
    idx = 0;
    for j=1:length(centeredPoints_2)
        try
            mahaldist = mean(mahal(centeredPoints_1{i}, centeredPoints_2{j}));
            barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
            if mahaldist < dist && barycenterDist < barycenterThreshold
                dist = mahaldist;
                idx = j;
            end
        catch
            warning('dimension problem');
        end
    end
    if idx~=0 && ~ismember(idx, idxList) && mahaldist ~= 0
        idxList = [idxList, idx];
        corespondences = [corespondences; [i, idx]];
    end
end
end

