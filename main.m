load KITTI_VEL_SCAN.mat

% point cloud analysis parameters
c_edge = 0.2;
c_plane = 0.05;
distThreshold = 0.2;
minClusterSize = 10;
barycenterThreshold = 1.5;


filteredCloud_1 = cloudFilter(traj{50}, "HDL64");
[edgeIdx_1, planeIdx_1, labelCloud_1, smoothnessCloud_1] = edgeDetector(filteredCloud_1.Location, c_edge, c_plane);

filteredCloud_2 = cloudFilter(traj{51}, "HDL64");
[edgeIdx_2, planeIdx_2, labelCloud_2, smoothnessCloud_2] = edgeDetector(filteredCloud_2.Location, c_edge, c_plane);

size1 = size(filteredCloud_1.Location, 1);
size2 = size(filteredCloud_1.Location, 2);




%----------------------------------------------------------------------
% evaluate the corespondence
%----------------------------------------------------------------------




% creating the edgeClouds

edgeCloud_1 = select(filteredCloud_1, ~edgeIdx_1, 'OutputSize', 'full');
edgeCloud_2 = select(filteredCloud_2, ~edgeIdx_2, 'OutputSize', 'full');

% clustering the edge clouds

[edgePoints_1, centeredEdgePoints_1, barycenterEdge_1, labelsEdge_1]...
    = clusteringCentering(edgeCloud_1, distThreshold, minClusterSize);
[edgePoints_2, centeredEdgePoints_2, barycenterEdge_2, labelsEdge_2]...
    = clusteringCentering(edgeCloud_2, distThreshold, minClusterSize);

% match the subclouds

corespondencesEdge = matching(centeredEdgePoints_1, centeredEdgePoints_2,...
    barycenterEdge_1, barycenterEdge_2, barycenterThreshold);


% creating the planeCloud 
% getting rid of the remaining ground around the car

idx = (filteredCloud_1.Location(:,:,1)>7) + (filteredCloud_1.Location(:,:,1)<-7);
idy = (filteredCloud_1.Location(:,:,2)>7) + (filteredCloud_1.Location(:,:,2)<-7);
index = logical((idx+idy));
index = logical(index.*(~planeIdx_1));
planeCloud_1 = select(filteredCloud_1, index, 'OutputSize', 'full');
idx = (filteredCloud_2.Location(:,:,1)>7) + (filteredCloud_2.Location(:,:,1)<-7);
idy = (filteredCloud_2.Location(:,:,2)>7) + (filteredCloud_2.Location(:,:,2)<-7);
index = logical((idx+idy));
index = logical(index.*(~planeIdx_2));
planeCloud_2 = select(filteredCloud_2, index, 'OutputSize', 'full');

% clustering the plane clouds

[planePoints_1, centeredPlane_1, barycenterPlane_1, labelsPlane_1, validLabels_1]...
    = clusteringCentering(planeCloud_1, 1, 10);
[planePoints_2, centeredPlane_2, barycenterPlane_2, labelsPlane_2, validLabels_2]...
    = clusteringCentering(planeCloud_2, 1, 10);

% match the plane clouds

corespondencesPlane = matching(centeredPlane_1, centeredPlane_2,...
    barycenterPlane_1, barycenterPlane_2, 5);


%--------------------------------------------------------------------------
% finding the correct rigid transform with Levenberg and Marquardt algorithm
%--------------------------------------------------------------------------


% finding dx, dy and dpsi with the edges

x0 = [0, 0, 0];
f = @(x)costEdge(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);

% remove outliers
firstEval = f(x0);
inliers = ~isoutlier(firstEval(:,1));
corespondencesEdge = corespondencesEdge(inliers,:);

%levenberg Marquardt optimisation 
lb = [-1.5, -1.5, -pi/3];
ub = [1.5, 1.5, pi/3];
f = @(x)costEdge_mahalanobis(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);
try
    options = optimoptions('lsqnonlin','FunctionTolerance', 0.001);
    [x, ~] = lsqnonlin(f,x0,lb,ub,options);
catch
    warning('optimisation failure')
end
disp(x);


% finding dz, dtheta and dpsi with the planes

y0 = [0,0,0];
f = @(x)costPlane(corespondencesPlane, barycenterPlane_1, barycenterPlane_2, x(1), x(2), x);

%remove outliers
firstEval = f(y0);
inliers = ~isoutlier(firstEval(:,1));
corespondencesPlane = corespondencesPlane(inliers,:);

%levenberg Marquardt optimisation
lb = [-1.5, -pi/3, -pi/3];
ub = [1.5, pi/3, pi/3];
f = @(x)costPlane(corespondencesPlane, barycenterPlane_1, barycenterPlane_2, x(1), x(2), x);
try
    options = optimoptions('lsqnonlin','FunctionTolerance', 0.001);
    [y, ~] = lsqnonlin(f,y0,lb,ub,options);
catch
    warning('optimisation failure')
end
disp(y);


%--------------------------------------------------------------------------
%display the results
%--------------------------------------------------------------------------






labelCorespondences_1 = nan(size1,size2);
labelCorespondences_2 = nan(size1,size2);
for i=1:size(corespondencesPlane,1)
    label1 = validLabels_1(corespondencesPlane(i,1));
    labelCorespondences_1(find(labelsPlane_1==label1)) = i;
    label2 = validLabels_2(corespondencesPlane(i,2));
    labelCorespondences_2(find(labelsPlane_2==label2)) = i;
end

figure(1)
pcshow(planeCloud_1.Location,labelCorespondences_1)
colormap(hsv(size(corespondencesPlane,1)))
title('plane 1 Matched')

figure(2)
pcshow(planeCloud_2.Location,labelCorespondences_2)
colormap(hsv(size(corespondencesPlane,1)))
title('plane 2 Matched')

figure(3)
pcshow(filteredCloud_1.Location,labelCloud_1)
colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
title('Point Cloud planes and edges')

figure(4)
pcshow(filteredCloud_2.Location,labelCloud_2)
colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
title('Point Cloud planes and edges')
