function [clusterArray, centeredCloud, barycenterMap, labels, validLabels] = clusteringCentering(ptCloud, distThreshold, minClusterSize)

[labels,numClusters] = pcsegdist(ptCloud,distThreshold);
clusterArray = {};
ct = 1;
validLabels = [];
% select only the biggest plane and generate the planePoint array
for i=1:numClusters
    if nnz(labels==i)>minClusterSize
        cluster = select(ptCloud, find(labels==i));
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

