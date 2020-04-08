% load KITTI_VEL_SCAN.mat

% point cloud analysis parameters
c_edge = 0.2;
c_plane = 0.05;
distThreshold = 0.2;
minClusterSize = 5;
barycenterThreshold = 1.5;


filteredCloud_1 = cloudFilter(traj{5}, "HDL64");
[edgeIdx_1, planeIdx_1, labelCloud_1, smoothnessCloud_1] = edgeDetector(filteredCloud_1.Location, c_edge, c_plane);

filteredCloud_2 = cloudFilter(traj{6}, "HDL64");
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

[corespondencesEdge, edgeWeights] = matchingEdge(centeredEdgePoints_1, centeredEdgePoints_2,...
    barycenterEdge_1, barycenterEdge_2, barycenterThreshold);


% creating the planeCloud 

planeCloud_1 = select(filteredCloud_1, ~planeIdx_1, 'OutputSize', 'full');
planeCloud_2 = select(filteredCloud_2, ~planeIdx_2, 'OutputSize', 'full');

% clustering the plane clouds

[planePoints_1, centeredPlane_1, barycenterPlane_1, labelsPlane_1, validLabels_1]...
    = clusteringCentering(planeCloud_1, 0.3, 50);
[planePoints_2, centeredPlane_2, barycenterPlane_2, labelsPlane_2, validLabels_2]...
    = clusteringCentering(planeCloud_2, 0.3, 50);

% create the normal array

[normalsPlane_1, normalsStd_1] = normalsGenerator(planePoints_1);
[normalsPlane_2, normalsStd_2]  = normalsGenerator(planePoints_2);


% match the plane clouds

[corespondencesPlane, planeWeights] = matchingPlane(centeredPlane_1, centeredPlane_2,...
    normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, 3);

%--------------------------------------------------------------------------
% finding the correct rigid transform with Levenberg and Marquardt algorithm
%--------------------------------------------------------------------------


% finding dx, dy and dpsi with the edges

x0 = [0, 0, 0];
f = @(x)costEdge(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);

% remove outliers
firstEval = f(x0);
inliers = ~isoutlier(firstEval);
inliers = logical(inliers(:,1).*inliers(:,2));
corespondencesEdge = corespondencesEdge(inliers,:);
edgeWeights = edgeWeights(inliers);

% finding dz, dtheta and dpsi with the planes

y0 = [0,0,0,0,0,0];
f = @(x)costPlane(corespondencesPlane, normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, x);

%remove outliers
firstEval = f(y0);
inliers = ~isoutlier(firstEval);
inliers = logical(inliers(:,1).*inliers(:,2));
corespondencesPlane = corespondencesPlane(inliers,:);
planeWeights = planeWeights(inliers);


% global levenberg Marquardt optimisation 
x0 = [0,0,0,0,0,0];
lb = [-1.5, -1.5, -0.5, -pi/6, -pi/6, -pi/6];
ub = [1.5, 1.5, 0.5, pi/6, pi/6, pi/6];
f = @(x)globalCost(corespondencesEdge, corespondencesPlane, barycenterEdge_1, barycenterEdge_2,...
    normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, edgeWeights, planeWeights, x);
try
    options = optimoptions('lsqnonlin','FunctionTolerance', 0.001);
    [x, ~] = lsqnonlin(f,x0,lb,ub,options);
catch
    warning('optimisation failure')
end
disp(x);


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
