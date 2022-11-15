function planeStruct = clusteringPlane(planeStruct, detector_params)
% segment the cloud and create all the arrays necessary for matching 

%% segmentation of the cloud

[labels,numClusters] = pcsegdist(planeStruct.planeCloud,...
    detector_params.distThresholdPlane);
planeStruct.planePoints = {};
ct = 1;
planeStruct.validLabels = [];
planeStruct.normalsPlane = [];
planeStruct.normalsStd = [];
planeStruct.eigen = [];
planeStruct.normalsList = {};
planeStruct.labels = labels;

%% select only the biggest clusters, generate the cluster array and the normals

for i=1:numClusters
    if nnz(labels==i)>detector_params.minClusterSizePlane
        cluster = select(planeStruct.planeCloud, find(labels==i));
        
        % check if it's not a cloud full of zeros
        if cluster.Location(1,1) == 0
            continue;
        end
       
        plane = cluster.Location;
        
        % analyse the normal of the plane
        
        normals = pcnormals(pointCloud(plane));
        
        % flip the normals to point in the same direction
        
        for k = 1:length(normals)
            p1 = [0,0,0] - plane(k,:);
            p2 = normals(k,:);
            angle = atan2(norm(cross(p1,p2)),p1*p2');
            if angle > pi/2 || angle < -pi/2
                normals(k,:) = -normals(k,:);
            end
        end
        
        % remove outliers
        inliers = ~isoutlier(normals);
        inliers = logical(inliers(:,1).*inliers(:,2).*inliers(:,3));
        normals = normals(inliers,:);
        plane = plane(inliers,:);         
        avgNormal = mean(normals)/norm(mean(normals));
        
        % check if the new plane is not empty
        if isempty(plane)
            continue
        end
        
        % check if it's a good plane
        svdPlane = eig(cov(plane));
        if svdPlane(3)/svdPlane(2) < 25
            continue
        end
        
        planeStruct.planePoints{ct} = plane;
        planeStruct.normalsList{ct} = normals;
        planeStruct.normalsPlane(:, ct) = avgNormal;
        planeStruct.normalsStd(:,ct) = std(normals);
        planeStruct.eigen(:,ct) = svdPlane;
        ct = ct+1;
        planeStruct.validLabels = [planeStruct.validLabels, i];
        
        
    else
        labels(labels==i)=0;
    end
end

%% create the barycenter map

planeStruct.barycenterMap = zeros(length(planeStruct.planePoints), 3);
for i=1:length(planeStruct.planePoints)
    planeStruct.barycenterMap(i,:) = barycenter(planeStruct.planePoints{i});
end

end

