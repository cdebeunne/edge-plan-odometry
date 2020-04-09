%load KITTI_VEL_SCAN.mat

% point cloud analysis parameters
c_edge = 0.2;
c_plane = 0.05;
distThresholdEdge = 0.2;
minClusterSizeEdge = 5;
barycenterThresholdEdge = 1.5;
distThresholdPlane = 0.3;
minClusterSizePlane = 50;
barycenterThresholdPlane = 3;

% estimator parameters

% estimation of x, y and psi
xWorld = [0;0;0];
posList = [xWorld];
tpp = [0,0,0];
theta = 0;
phi = 0;
psi = 0;
R = eye(3,3);

for k=1:size(traj,2)-1
    disp(k);
    
    % in case of empty clouds 
    if size(traj{k},1) == 0 || size(traj{k+1},1)==0
        dxWorld = R*dxWorld;
        xWorld = xWorld + dxWorld;
        posList = [posList, xWorld];
        continue
    end
    
    % first filtering
    filteredCloud_1 = cloudFilter(traj{k},"HDL64");
    [edgeIdx_1, planeIdx_1, labelCloud_1, smoothnessCloud_1] = edgeDetector(filteredCloud_1.Location, c_edge, c_plane);
    
    filteredCloud_2 = cloudFilter(traj{k+1},"HDL64");
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
   


    % creating the plane clouds
    planeCloud_1 = select(filteredCloud_1, ~planeIdx_1, 'OutputSize', 'full');
    planeCloud_2 = select(filteredCloud_2, ~planeIdx_2, 'OutputSize', 'full');
    
    % clustering the plane clouds
    [planePoints_1, centeredPlane_1, barycenterPlane_1, labelsPlane_1, validLabels_1]...
        = clusteringCentering(planeCloud_1, distThresholdPlane, minClusterSizePlane);
    [planePoints_2, centeredPlane_2, barycenterPlane_2, labelsPlane_2, validLabels_2]...
        = clusteringCentering(planeCloud_2, distThresholdPlane, minClusterSizePlane);
    
    % creating the normal arrays
    [normalsPlane_1, normalsStd_1] = normalsGenerator(planePoints_1);
    [normalsPlane_2, normalsStd_2]  = normalsGenerator(planePoints_2);
    
    % match the plane clouds
    [corespondencesPlane, planeWeights] = matchingPlane(centeredPlane_1, centeredPlane_2,...
    normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, 3);
    


    %--------------------------------------------------------------------------
    % finding the correct rigid transform with Levenberg and Marquardt algorithm
    %--------------------------------------------------------------------------
    
    
    
    x0 = [0, 0, 0];
    f = @(x)costEdge(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);
    
    % remove outliers
    firstEval = f(x0);
    inliers = ~isoutlier(firstEval);
    inliers = logical(inliers(:,1).*inliers(:,2));
    corespondencesEdge = corespondencesEdge(inliers,:);
    
    y0 = [0,0,0,0,0,0];
    f = @(x)costPlane(corespondencesPlane, normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, x);
    
    %remove outliers
    firstEval = f(y0);
    inliers = ~isoutlier(firstEval);
    inliers = logical(inliers(:,1).*inliers(:,2));
    corespondencesPlane = corespondencesPlane(inliers,:);
    
    
    % global levenberg Marquardt optimisation
    x0 = [0,0,0,0,0,0];
    lb = [-1.5, -1.5, -0.5, -pi/6, -pi/6, -pi/6];
    ub = [1.5, 1.5, 0.5, pi/6, pi/6, pi/6];
    f = @(x)globalCost(corespondencesEdge, corespondencesPlane, barycenterEdge_1, barycenterEdge_2,...
        normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, edgeWeights, planeWeights, x);
    try
        options = optimoptions('lsqnonlin','FunctionTolerance', 0.01);
        [x, ~] = lsqnonlin(f,x0,lb,ub,options);
    catch
        warning('optimisation failure')
    end
    
    
    
    %----------------------------------------------------------------------
    % adding the new pose in world coordinates
    %----------------------------------------------------------------------
    
    
    
    % x,y and psi
    theta = theta + x(4);
    phi = phi + x(5);
    psi = psi + x(6);
    R = eul2rotm([theta, phi, psi], 'XYZ');
    dxWorld = R*x(1:3)';
    xWorld = xWorld + dxWorld;
    posList = [posList, xWorld];
    tpp = [tpp; [theta, phi, psi]];
end

load KITTI_OSTX.mat

% display the results

plot(posList(1,:), posList(2,:));
hold on;
plot(groundtruth(:,1),groundtruth(:,2));
legend('Edge & plane odometry', 'Groundtruth');