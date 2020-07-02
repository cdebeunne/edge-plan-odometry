function edgeStruct = clusteringEdge(edgeStruct, detector_params)
% segment the cloud and create all the arrays necessary for matching 

%% segmentation of the cloud 
[labels,numClusters] = pcsegdist(edgeStruct.edgeCloud,...
    detector_params.distThresholdEdge);
edgeStruct.edgePoints = {};
edgeStruct.directions = [];
edgeStruct.eigen = [];
ct = 1;
edgeStruct.validLabels = [];

%% select only the biggest clusters, generate the cluster array and the direction

for i=1:numClusters
    if nnz(labels==i)>detector_params.minClusterSizeEdge
        cluster = select(edgeStruct.edgeCloud, find(labels==i));
        
        % check if it's not a cloud full of zeros
        if cluster.Location(1,1) == 0
            continue;
        end
        edge = cluster.Location;
        
        % check if it's a good edge
        [v,e] = eig(cov(edge));
        direction = v(:,3);
        if direction(3) < 0.5 || e(3,3)/e(2,2)<5
            continue
        end
        
        
        edgeStruct.directions(:,ct) = direction;
        edgeStruct.eigen(:,ct) = [e(1,1); e(2,2); e(3,3)];
        edgeStruct.edgePoints{ct} = cluster.Location;
        ct = ct+1;
        edgeStruct.validLabels = [edgeStruct.validLabels, i];
    else
        labels(labels==i)=0;
    end
end



%% create the barycenter map

edgeStruct.barycenterMap = zeros(length(edgeStruct.edgePoints), 3);
for i=1:length(edgeStruct.edgePoints)
    edgeStruct.barycenterMap(i,:) = barycenter(edgeStruct.edgePoints{i});
end

end

