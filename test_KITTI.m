% Load data if requiered
close all
if ~exist('traj', 'var')
    load KITTI_VEL_SCAN.mat;
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

% estimator parameters
x = zeros(1,6);
poseStruct.xWorld = zeros(6,1);
poseStruct.posList = [poseStruct.xWorld];
poseStruct.transList = [x'];
poseStruct.R = eye(3,3);

for k=1:size(traj,2)-1
    disp(k);
    
    % in case of empty clouds
    
    if size(traj{k},1) == 0 || size(traj{k+1},1)==0
        % attitude update
        
        poseStruct.xWorld(4:6) = poseStruct.xWorld(4:6)+x(4:6);
        poseStruct.R = eul2rotm(poseStruct.xWorld(4:6), 'XYZ');
        
        % pose update
        
        poseStruct.dxWorld = R*x(1:3)';
        poseStruct.xWorld(1:3) = poseStruct.xWorld(1:3) + dxWorld;
        
        % storage
        
        poseStruct.posList = [poseStruct.posList, xWorld];
        poseStruct.transList = [poseStruct.transList, x'];
        continue
    end
    
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
        [x,resnorm,residual,exitflag,output,lambda,jacobian] = lsqnonlin(f,x,lb,ub,options);
    catch
        warning('optimisation failure')
    end
    
    disp(x);
    
    
    %----------------------------------------------------------------------
    % adding the new pose in world coordinates
    %----------------------------------------------------------------------
    
    % attitude update
    
    poseStruct.xWorld(4:6) = poseStruct.xWorld(4:6)+x(4:6)';
    poseStruct.R = eul2rotm(poseStruct.xWorld(4:6)', 'XYZ');
    
    % pose update
    
    poseStruct.dxWorld = poseStruct.R*x(1:3)';
    poseStruct.xWorld(1:3) = poseStruct.xWorld(1:3) + poseStruct.dxWorld;
    
    % storage
    
    poseStruct.posList = [poseStruct.posList, poseStruct.xWorld];
    poseStruct.transList = [poseStruct.transList, x'];
end

if ~exist('groundtruth', 'var')
    load KITTI_OSTX.mat;
end

% display the results

figure(1);
plot(posList(1,:), posList(2,:));
hold on;
axis equal;
plot(groundtruth(:,1),groundtruth(:,2));
xlabel('x (m)');
ylabel('y (m)');
legend('Edge & plane odometry', 'Groundtruth');
title('Position comparison');

figure(2)
plot(tpp(:,1));
hold on;
plot(att(:,1));
legend('Edge & plane odometry', 'Groundtruth');
title('Attitude comparison');

% save results

save('results.mat', 'tpp', 'posList', 'transList');