function [clusterArray, centeredCloud, barycenterMap, labels, validLabels]...
    = clusteringCentering(ptCloud, distThreshold, minClusterSize)
% segment the cloud and create all the arrays necessary for matching 

% segmentation of the cloud 
[labels,numClusters] = pcsegdist(ptCloud,distThreshold);
clusterArray = {};
ct = 1;
validLabels = [];

% select only the biggest clusters and generate the cluster array

for i=1:numClusters
    if nnz(labels==i)>minClusterSize
        cluster = select(ptCloud, find(labels==i));
        
        % check if it's not a cloud full of zeros
        if cluster.Location(1,1) == 0
            continue;
        end
        
        clusterArray{ct} = cluster.Location;
        ct = ct+1;
        validLabels = [validLabels, i];
    else
        labels(labels==i)=0;
    end
end

% create the centered array and the barycenter map

centeredCloud = {};
barycenterMap = zeros(length(clusterArray), 3);
for i=1:length(clusterArray)
    barycenterMap(i,:) = barycenter(clusterArray{i});
    centeredCloud{i} = clusterArray{i}-barycenterMap(i,:);
end

end

