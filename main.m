% load KITTI_VEL_SCAN.mat

% point cloud analysis parameters
c_edge = 0.15;
c_plane = 0.08;
distThresholdEdge = 0.3;
minClusterSizeEdge = 5;
barycenterThresholdEdge = 1.5;
distThresholdPlane = 0.3;
minClusterSizePlane = 30;
barycenterThresholdPlane = 3;

% first filtering of the clouds
filteredCloud_1 = cloudFilter(traj{1}, "HDL64");
[edgeIdx_1, planeIdx_1, labelCloud_1, smoothnessCloud_1] = edgePlaneDetector(filteredCloud_1.Location, c_edge, c_plane);

filteredCloud_2 = cloudFilter(traj{2}, "HDL64");
[edgeIdx_2, planeIdx_2, labelCloud_2, smoothnessCloud_2] = edgePlaneDetector(filteredCloud_2.Location, c_edge, c_plane);

size1 = size(filteredCloud_1.Location, 1);
size2 = size(filteredCloud_1.Location, 2);




%----------------------------------------------------------------------
% evaluate the corespondence
%----------------------------------------------------------------------




% creating the edgeClouds

edgeCloud_1 = select(filteredCloud_1, ~edgeIdx_1, 'OutputSize', 'full');
edgeCloud_2 = select(filteredCloud_2, ~edgeIdx_2, 'OutputSize', 'full');

% clustering the edge clouds

[edgePoints_1, barycenterEdge_1, directionsEdge_1, eigenEdge_1, labelsEdge_1, validEdge_1]...
    = clusteringEdge(edgeCloud_1, distThresholdEdge, minClusterSizeEdge);
[edgePoints_2, barycenterEdge_2, directionsEdge_2, eigenEdge_2, labelsEdge_2, validEdge_2]...
    = clusteringEdge(edgeCloud_2, distThresholdEdge, minClusterSizeEdge);

% match the subclouds

corespondencesEdge = matchingEdge(edgePoints_1, edgePoints_2,...
    barycenterEdge_1, barycenterEdge_2, eigenEdge_1, eigenEdge_2, barycenterThresholdEdge);

% creating the planeCloud 

planeCloud_1 = select(filteredCloud_1, ~planeIdx_1, 'OutputSize', 'full');
planeCloud_2 = select(filteredCloud_2, ~planeIdx_2, 'OutputSize', 'full');

% clustering the plane clouds

[planePoints_1, barycenterPlane_1, normalsPlane_1,...
    normalsStd_1, normalsList_1, eigenPlane_1, labelsPlane_1, validLabels_1]...
    = clusteringPlane(planeCloud_1, distThresholdPlane, minClusterSizePlane);
[planePoints_2, barycenterPlane_2, normalsPlane_2,...
    normalsStd_2, normalsList_2, eigenPlane_2, labelsPlane_2, validLabels_2]...
    = clusteringPlane(planeCloud_2, distThresholdPlane, minClusterSizePlane);


% match the plane clouds

corespondencesPlane = matchingPlane(planePoints_1, planePoints_2,...
    eigenPlane_1, eigenPlane_2, barycenterPlane_1, barycenterPlane_2, barycenterThresholdPlane);



%--------------------------------------------------------------------------
% finding the correct rigid transform with Levenberg and Marquardt algorithm
%--------------------------------------------------------------------------



% outliers rejection - edge

x0 = [0, 0, 0];
f = @(x)costEdge(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);

% remove outliers
firstEval = f(x0);
inliers = ~isoutlier(firstEval);
inliers = logical(inliers(:,1).*inliers(:,2));
corespondencesEdge = corespondencesEdge(inliers,:);

% outliers rejection - plane

y0 = [0,0,0,0,0,0];
f = @(x)costPlane(corespondencesPlane, normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, x);

%remove outliers
firstEval = f(y0);
inliers = ~isoutlier(firstEval);
inliers = logical(inliers(:,1).*inliers(:,2));
corespondencesPlane = corespondencesPlane(inliers,:);



% global levenberg Marquardt optimisation 
x0 = [0,0,0,0,0,0];
lb = [-1.5, -0.05, -0.02, -0.01, -0.01, -pi/6];
ub = [1.5, 0.05, 0.02, 0.01, 0.01, pi/6];
f = @(x)globalCost_orth(corespondencesEdge, corespondencesPlane,...
    edgePoints_1, directionsEdge_1, barycenterEdge_2, directionsEdge_2,...
    normalsPlane_1, normalsPlane_2, x);
try
    options = optimoptions('lsqnonlin','FunctionTolerance', 0.001, 'MaxFunctionEvaluations', 1000);
    [x,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(f,x0,lb,ub,options);
catch
    warning('optimisation failure')
end
disp(x);



%--------------------------------------------------------------------------
%display the results
%--------------------------------------------------------------------------


labelCorespondences_1 = nan(size1,size2);
labelCorespondences_2 = nan(size1,size2);
for i=1:length(corespondencesEdge)
    label1 = validEdge_1(corespondencesEdge(i,1));
    labelCorespondences_1(find(labelsEdge_1==label1)) = i;
    label2 = validEdge_2(corespondencesEdge(i,2));
    labelCorespondences_2(find(labelsEdge_2==label2)) = i;
end

figure(1)
pcshow(edgeCloud_1.Location,labelCorespondences_1, 'MarkerSize', 30)
colormap(hsv(size(corespondencesEdge,1)))
title('edge Matched')

figure(2)
pcshow(edgeCloud_2.Location,labelCorespondences_2, 'MarkerSize', 30)
colormap(hsv(size(corespondencesEdge,1)))
% 
% labelCorespondences_1 = nan(size1,size2);
% labelCorespondences_2 = nan(size1,size2);
% for i=1:size(corespondencesPlane,1)
%     label1 = validLabels_1(corespondencesPlane(i,1));
%     labelCorespondences_1(find(labelsPlane_1==label1)) = i;
%     label2 = validLabels_2(corespondencesPlane(i,2));
%     labelCorespondences_2(find(labelsPlane_2==label2)) = i;
% end
% 
% figure;
% pcshow(planeCloud_1.Location,labelCorespondences_1)
% colormap(hsv(size(corespondencesPlane,1)))
% title('plane 1 Matched')
% 
% figure;
% pcshow(planeCloud_2.Location,labelCorespondences_2)
% colormap(hsv(size(corespondencesPlane,1)))
% title('plane 2 Matched')
% 
% figure(3)
% pcshow(filteredCloud_1.Location,labelCloud_1)
% colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
% title('Point Cloud planes and edges')
% 
% figure(4)
% pcshow(filteredCloud_2.Location,labelCloud_2)
% colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
% title('Point Cloud planes and edges')
