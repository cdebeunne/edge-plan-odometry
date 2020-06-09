function corespondencesEdge = matchingEdge(centeredPoints_1,...
    centeredPoints_2,...
    barycenterMap_1, barycenterMap_2,...
    eigenEdge_1, eigenEdge_2,...
    barycenterThreshold)
% matching the edges and creating the weight array

corespondencesEdge = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(centeredPoints_1)
    dist = 1e+33;
    idx = 0;
    for j=1:length(centeredPoints_2)
        try
%             mahaldist = mean(mahal(centeredPoints_1{i}, centeredPoints_2{j}));
            eigenDist = norm(eigenEdge_1(:,i)-eigenEdge_2(:,j));
            barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
            deltaSize = abs(length(centeredPoints_1{i})-length(centeredPoints_2{j}))/...
                max(length(centeredPoints_1{i}),length(centeredPoints_2{j}));
            
            if eigenDist < dist && barycenterDist < barycenterThreshold && deltaSize<0.2
                dist = eigenDist;
                idx = j;
            end
        catch
            warning('dimension problem');
        end
    end
    if idx~=0 && ~ismember(idx, idxList) 
        idxList = [idxList, idx];
        corespondencesEdge = [corespondencesEdge; [i, idx]];
    end
end

end

