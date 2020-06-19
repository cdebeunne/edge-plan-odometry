%load KITTI_VEL_SCAN2.mat
% load FILT_IMU_DAT.mat;
imuTime = filtered_ROSTime;
imuACC = filtered_ACC;
imuGYR = filtered_GYR;
time = filtered_scantime;

% point cloud analysis parameters
c_edge = 0.2;
c_plane = 0.05;
distThresholdEdge = 0.6;
minClusterSizeEdge = 5;
barycenterThresholdEdge = 1.5;
distThresholdPlane = 1;
minClusterSizePlane = 30;
barycenterThresholdPlane = 10;


%intialisation of the estimator
X = zeros(12,1);
P = zeros(12, 12);
P(1:6, 1:6) = zeros(6,6);
Q = [0.1*eye(3,3) zeros(3,9);
    zeros(3,3), 1e-4*eye(3,3), zeros(3,6);
    zeros(6,6) 0.00001*eye(6,6)];
transV = zeros(3,1);
fail = 0;
err = [];

x = [0,0,0,0,0,0];
xWorld = [0;0;0];
posList = [xWorld];
theta = 0;
phi = 0;
psi = 0;
tpp = [theta, phi, psi];
R = eye(3,3);

for k=1:1200
    disp(k);
    
    % in case of empty clouds
    if size(filtered_traj{k},1) == 0 || size(filtered_traj{k+1},1)==0
        theta = theta + X(4);
        phi = phi + X(5);
        psi = psi + X(6);
        R = eul2rotm([theta, phi, psi], 'XYZ');
        dxWorld = R*X(1:3)';
        xWorld = xWorld + dxWorld;
        posList = [posList, xWorld];
        tpp = [tpp; [theta, phi, psi]];
        continue
    end
    
    % first filtering
    filteredCloud_1 = cloudFilter(filtered_traj{k},"VLP16");
    [edgeIdx_1, planeIdx_1, labelCloud_1, smoothnessCloud_1] =...
        edgePlaneDetector(filteredCloud_1.Location, c_edge, c_plane);
    %get IMU's timestamp
    t0 = time(k);
    nearest_index0 = interp1(imuTime, 1:length(imuTime), t0, 'nearest');
    
    filteredCloud_2 = cloudFilter(filtered_traj{k+1},"VLP16");
    [edgeIdx_2, planeIdx_2, labelCloud_2, smoothnessCloud_2] =...
        edgePlaneDetector(filteredCloud_2.Location, c_edge, c_plane);
    %get IMU's timestamp
    t1 = time(k+1);
    nearest_index1 = interp1(imuTime, 1:length(imuTime), t1, 'nearest');
    
    if ~isnan(nearest_index0) && t1-t0<1
        [transVecImu, angVecImu, transV] = imuAnalyser(imuTime, imuACC, imuGYR,...
            t0, t1, nearest_index0, nearest_index1, transV);
    end
    
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
        normalsStd_1, normalsList_1, labelsPlane_1, validLabels_1]...
        = clusteringPlane(planeCloud_1, distThresholdPlane, minClusterSizePlane);
    [planePoints_2, barycenterPlane_2, normalsPlane_2,...
        normalsStd_2, normalsList_2, labelsPlane_2, validLabels_2]...
        = clusteringPlane(planeCloud_2, distThresholdPlane, minClusterSizePlane);
    
    
    % match the plane clouds
    
    corespondencesPlane = matchingPlane(planePoints_1, planePoints_2,...
        normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, barycenterThresholdPlane);
    
    
    
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
    try
        corespondencesEdge = corespondencesEdge(inliers,:);
    catch
        warning('yoy');
        err = [err;k];
    end
    
    y0 = [0,0,0,0,0,0];
    f = @(x)costPlane(corespondencesPlane, normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, x);
    
    %remove outliers
    firstEval = f(y0);
    inliers = ~isoutlier(firstEval);
    inliers = logical(inliers(:,1).*inliers(:,2));
    corespondencesPlane = corespondencesPlane(inliers,:);
    
    % optimisation
    
    x0 = [0,0,0,0,0,0];
    lb = [-1.5, -0.05, -0.02, -0.01, -0.01, -pi/6];
    ub = [1.5, 0.05, 0.02, 0.01, 0.01, pi/6];
    f = @(x)globalCost_orth(corespondencesEdge, corespondencesPlane,...
    edgePoints_1, directionsEdge_1, barycenterEdge_2, directionsEdge_2,...
    normalsPlane_1, normalsPlane_2, x);
    
    try
        options = optimoptions('lsqnonlin','FunctionTolerance', 0.001, 'MaxFunctionEvaluations', 1000);
        [x, ~] = lsqnonlin(f,x,lb,ub,options);
        fail = 0;
    catch
        warning('optimisation failure')
        fail = 1;
        err = [err; k];
    end
    
    %----------------------------------------------------------------------
    % Fusion with IMU thanks to Kalman Filter
    %----------------------------------------------------------------------
    
    if fail == 0
        u = [transVecImu; angVecImu];
        z = x';
        L = [1e-2*eye(3,3), zeros(3,3);
            zeros(3,3), 0.1*eye(3,3)];
        [X, P] = kalmanFilter(X, P, u, z, Q, L);
    else
        X = [transVecImu; angVecImu; X(7:12)];
    end
    
    %----------------------------------------------------------------------
    % adding the new pose in world coordinates
    %----------------------------------------------------------------------  
    
    % x,y and psi
    theta = theta + X(4);
    phi = phi + X(5);
    psi = psi + X(6);
    R = eul2rotm([theta, phi, psi], 'XYZ');
    dxWorld = R*X(1:3);
    xWorld = xWorld + dxWorld;
    posList = [posList, xWorld];
    tpp = [tpp; [theta, phi, psi]];
    transV = X(1:3)./(t1-t0);
    
    disp(X');
end

% load KITTI_OSTX.mat

pos = groundtruth(filtered_posEnu, -3.2*pi/10);

% display the results

figure(1);
plot(posList(1,:), posList(2,:));
hold on;
plot(pos(:,1),pos(:,2));
axis equal;
legend('Edge & plane odometry', 'Groundtruth');
title('Position comparison');

% save results

save('results.mat', 'tpp', 'posList');