function corespondencesPlane = matchingPlane(planePoints_1,...
    planePoints_2,... 
    eigenPlane_1, eigenPlane_2,...
    barycenterMap_1, barycenterMap_2,...
    barycenterThreshold)
% matching the planes and creating the weight array 

corespondencesPlane = [];
% idenx list to prevent from double match
idxList = [];  
for i=1:length(planePoints_1)
    dist = 5;
    idx = 0;
    for j=1:length(planePoints_2)
        try
            eigenDist = norm(eigenPlane_1(:,i)-eigenPlane_2(:,j));
            barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
            deltaSize = abs(length(planePoints_1{i})-length(planePoints_2{j}))/...
                max(length(planePoints_1{i}),length(planePoints_2{j})); 
            if eigenDist < dist && barycenterDist < barycenterThreshold && deltaSize < 0.5
                dist = eigenDist;
                idx = j;
            end
        catch
            warning('dimension problem');
        end
    end
    if idx~=0 
        corespondencesPlane = [corespondencesPlane; [i, idx]];
    end
end

end
