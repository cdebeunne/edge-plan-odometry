function corespondencesPlane = matchingPlane(planeStruct_1,...
    planeStruct_2, detector_params)
% matching the planes and creating the weight array 
 %% initialization
 
corespondencesPlane = [];

%% plane checking

% idenx list to prevent from double match
idxList = [];  
for i=1:length(planeStruct_1.planePoints)
    dist = 5;
    idx = 0;
    for j=1:length(planeStruct_2.planePoints)
        try
            eigenDist = norm(planeStruct_1.eigen(:,i)-...
                planeStruct_2.eigen(:,j));
            barycenterDist = norm(planeStruct_1.barycenterMap(i,1:2)-...
                planeStruct_2.barycenterMap(j,1:2));
            deltaSize = abs(length(planeStruct_1.planePoints{i})-...
                length(planeStruct_2.planePoints{j}))/...
                max(length(planeStruct_1.planePoints{i}),...
                length(planeStruct_2.planePoints{j})); 
            if eigenDist < dist &&...
                    barycenterDist < detector_params.barycenterThresholdPlane...
                    && deltaSize < 0.5
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
