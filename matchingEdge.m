function corespondencesEdge = matchingEdge(edgePoints_1,...
    edgePoints_2,...
    barycenterMap_1, barycenterMap_2,...
    eigenEdge_1, eigenEdge_2,...
    barycenterThreshold)
% matching the edges and creating the weight array

corespondencesEdge = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(edgePoints_1)
    dist = 6;
    idx = 0;
    for j=1:length(edgePoints_2)
        try
            eigenDist = norm(eigenEdge_1(:,i)-eigenEdge_2(:,j));
            barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
            deltaSize = abs(length(edgePoints_1{i})-length(edgePoints_2{j}))/...
                max(length(edgePoints_1{i}),length(edgePoints_2{j}));
            
            if eigenDist < dist && barycenterDist < barycenterThreshold && deltaSize<0.2
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

