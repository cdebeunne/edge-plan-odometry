function [normalsPlane, normalsStd, normalsList]= normalsGenerator(planePoints)
% build the normal map for plane analysis

normalsPlane = zeros(3, length(planePoints));
normalsStd = zeros(3, length(planePoints));
normalsList = cell(length(planePoints),1);

for i=1:length(planePoints)
    normals = pcnormals(pointCloud(planePoints{i}));
    
    
    % flip the normals to point in the same direction
    
    for k = 1:length(normals)
        p1 = [0,0,0] - planePoints{i}(k,:);
        p2 = normals(k,:);
        angle = atan2(norm(cross(p1,p2)),p1*p2');
        if angle > pi/2 || angle < -pi/2
            normals(k,:) = -normals(k,:);
        end
    end
    normalsList{i} = normals;
    
    % remove outliers 
    inliers = ~isoutlier(normals);
    inliers = logical(inliers(:,1).*inliers(:,2).*inliers(:,3));
    normals = normals(inliers,:);
    
    
    normalsPlane(:,i) = mean(normals)/norm(mean(normals));
    normalsStd(:,i) = std(normals);
end