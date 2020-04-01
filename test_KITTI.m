load KITTI_VEL_SCAN.mat

% point cloud analysis parameters
c_edge = 0.2;
c_plane = 0.05;
distThreshold = 0.3;
minClusterSize = 10;
barycenterThreshold = 1.5;

% estimator parameters

% estimation of x, y and psi
xWorld = [0;0];
posList = [xWorld];
psi = 0;
R = eye(2,2);

% estimation of z, theta and psi
ztp = [0,0,0];

for k=1:size(traj,2)-1
    disp(k);
    if size(traj{k},1) == 0 || size(traj{k+1},1)==0
        dxWorld = R*dxWorld;
        xWorld = xWorld + dxWorld;
        posList = [posList, xWorld];
        continue
    end
    
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
    
    % finding dx, dy, dpsi with the edge 
    
    x0 = [0, 0, 0];
    f = @(x)costEdge(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);
    
    % remove outliers
    firstEval = f(x0);
    inliers = ~isoutlier(firstEval(:,1));
    corespondencesEdge = corespondencesEdge(inliers,:);
    
    %levenberg Marquardt optimisation
    lb = [-1.5, -1.5, -pi/3];
    ub = [1.5, 1.5, pi/3];
    f = @(x)costEdge_mahalanobis(corespondencesEdge, edgePoints_1, edgePoints_2, x);
    try
        options = optimoptions('lsqnonlin','FunctionTolerance', 0.001);
        [x, ~] = lsqnonlin(f,x0,lb,ub,options);
    catch
        warning('optimisation failure')
    end
    
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
    
    
    %----------------------------------------------------------------------
    % adding the new pose in world coordinates
    %----------------------------------------------------------------------
    
    % x,y and psi
    psi = psi + x(3);
    R = [cos(psi) -sin(psi); sin(psi) cos(psi)];
    dxWorld = R*x(1:2)';
    xWorld = xWorld + dxWorld;
    posList = [posList, xWorld];
    
    % z, theta and psi
    new_ztp = ztp(end,:)+y;
    ztp = [ztp;new_ztp];
end

load KITTI_OSTX.mat

% display the results

plot(posList(1,:), posList(2,:));
hold on;
plot(groundtruth(:,1),groundtruth(:,2));
legend('Edge odometry', 'Groundtruth');