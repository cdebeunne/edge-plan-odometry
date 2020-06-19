function [edgePoints, barycenterMap, directions, eigen, labels, validLabels]...
    = clusteringEdge(ptCloud, distThreshold, minClusterSize)
% segment the cloud and create all the arrays necessary for matching 

%% segmentation of the cloud 
[labels,numClusters] = pcsegdist(ptCloud,distThreshold);
edgePoints = {};
directions = [];
eigen = [];
ct = 1;
validLabels = [];

%% select only the biggest clusters, generate the cluster array and the direction

for i=1:numClusters
    if nnz(labels==i)>minClusterSize
        cluster = select(ptCloud, find(labels==i));
        
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
        
        
        directions(:,ct) = direction;
        eigen(:,ct) = [e(1,1); e(2,2); e(3,3)];
        edgePoints{ct} = cluster.Location;
        ct = ct+1;
        validLabels = [validLabels, i];
    else
        labels(labels==i)=0;
    end
end



%% create the barycenter map

barycenterMap = zeros(length(edgePoints), 3);
for i=1:length(edgePoints)
    barycenterMap(i,:) = barycenter(edgePoints{i});
end

end

