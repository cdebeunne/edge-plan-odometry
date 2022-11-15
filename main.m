% Load data if requiered
close all 
if ~exist('traj', 'var')
    load KITTI_VEL_SCAN.mat
end

% point cloud analysis parameters
detector_params.c_edge = 0.1;
detector_params.c_plane = 0.05;
detector_params.distThresholdEdge = 0.3;
detector_params.minClusterSizeEdge = 5;
detector_params.barycenterThresholdEdge = 1.5;
detector_params.distThresholdPlane = 0.3;
detector_params.minClusterSizePlane = 30;
detector_params.barycenterThresholdPlane = 3;

% first filtering of the clouds
filteredCloud_1 = cloudFilter(traj{20}, "HDL64");
[edgeStruct_1.edgeIdx, planeStruct_1.planeIdx,...
    labelCloud_1, smoothnessCloud_1] =...
    edgePlaneDetector(filteredCloud_1.Location, detector_params);

filteredCloud_2 = cloudFilter(traj{21}, "HDL64");
[edgeStruct_2.edgeIdx, planeStruct_2.planeIdx,...
    labelCloud_2, smoothnessCloud_2] =...
    edgePlaneDetector(filteredCloud_2.Location, detector_params);


%----------------------------------------------------------------------
% evaluate the corespondence
%----------------------------------------------------------------------


% creating the edgeClouds

edgeStruct_1.edgeCloud = select(filteredCloud_1, ~edgeStruct_1.edgeIdx,...
    'OutputSize', 'full');
edgeStruct_2.edgeCloud = select(filteredCloud_2, ~edgeStruct_2.edgeIdx,...
    'OutputSize', 'full');

% clustering the edge clouds

edgeStruct_1 = clusteringEdge(edgeStruct_1, detector_params);
edgeStruct_2 = clusteringEdge(edgeStruct_2, detector_params);

% match the subclouds

corespondencesEdge = matchingEdge(edgeStruct_1, edgeStruct_2, detector_params);

% creating the planeCloud 

planeStruct_1.planeCloud = select(filteredCloud_1, ~planeStruct_1.planeIdx,...
    'OutputSize', 'full');
planeStruct_2.planeCloud = select(filteredCloud_2, ~planeStruct_2.planeIdx,...
    'OutputSize', 'full');

% clustering the plane clouds

planeStruct_1 = clusteringPlane(planeStruct_1, detector_params);
planeStruct_2 = clusteringPlane(planeStruct_2, detector_params);


% match the plane clouds

corespondencesPlane = matchingPlane(planeStruct_1,...
    planeStruct_2, detector_params);



%--------------------------------------------------------------------------
% finding the correct rigid transform with Levenberg and Marquardt algorithm
%--------------------------------------------------------------------------


% outliers rejection - edge

x0 = [0, 0, 0];
f = @(x)costEdge(corespondencesEdge, edgeStruct_1, edgeStruct_2, x);

% remove outliers

firstEval = f(x0);
inliers = ~isoutlier(firstEval);
inliers = logical(inliers(:,1).*inliers(:,2));
corespondencesEdge = corespondencesEdge(inliers,:);

% global levenberg Marquardt optimisation 

x0 = [0,0,0,0,0,0];
lb = [-1.5, -0.05, -0.02, -0.01, -0.01, -pi/6];
ub = [1.5, 0.05, 0.02, 0.01, 0.01, pi/6];
f = @(x)globalCost_bary(corespondencesEdge, corespondencesPlane,...
                                edgeStruct_1, edgeStruct_2, ...
                                planeStruct_1, planeStruct_2, x);
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


size1 = size(filteredCloud_1.Location, 1);
size2 = size(filteredCloud_1.Location, 2);

labelCorespondences_1 = nan(size1,size2);
labelCorespondences_2 = nan(size1,size2);

for i=1:length(corespondencesEdge)
    label1 = corespondencesEdge(i,1);
    labelCorespondences_1(find(edgeStruct_1.labels==edgeStruct_1.validLabels(label1))) = i;
    label2 = corespondencesEdge(i,2);
    labelCorespondences_2(find(edgeStruct_2.labels==edgeStruct_2.validLabels(label2))) = i;
end

figure(1)
pcshow(edgeStruct_1.edgeCloud.Location,labelCorespondences_1, 'MarkerSize', 80)
colormap(hsv(size(corespondencesEdge,1)))
title('edge Matched')

labelCorespondences_1 = nan(size1,size2);
labelCorespondences_2 = nan(size1,size2);
for i=1:size(corespondencesPlane,1)
    label1 = corespondencesPlane(i,1);
    labelCorespondences_1(find(planeStruct_1.labels==planeStruct_1.validLabels(label1))) = i;
    label2 = corespondencesPlane(i,2);
    labelCorespondences_2(find(planeStruct_2.labels==planeStruct_2.validLabels(label2))) = i;
end

figure;
pcshow(planeStruct_1.planeCloud.Location,labelCorespondences_1)
colormap(hsv(size(corespondencesPlane,1)))
title('plane 1 Matched')

figure;
pcshow(planeStruct_2.planeCloud.Location,labelCorespondences_2)
colormap(hsv(size(corespondencesPlane,1)))
title('plane 2 Matched')

figure(3)
pcshow(filteredCloud_1.Location,labelCloud_1, 'MarkerSize', 20)
colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
title('Point Cloud planes and edges')

figure(4)
pcshow(filteredCloud_2.Location,labelCloud_2, 'MarkerSize', 30)
colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
title('Point Cloud planes and edges')
