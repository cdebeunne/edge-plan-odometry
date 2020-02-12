load VEL_SCAN_DAT.mat
load SBG_IMU_DATA.mat;
imuTime = SBG_IMU_DATA.ROStime;
imuACC = SBG_IMU_DATA.ACC;
imuACC = [imuACC(:,1), imuACC(:,2), imuACC(:,3)];
imuGYR = SBG_IMU_DATA.GYR;

% point cloud analysis parameters
c_edge = 1;
c_plane = 0.025;
distThreshold = 0.5;
minClusterSize = 10;
barycenterThreshold = 0.3;


%intialisation of the estimator
X = [0; 0; 0; 0; 0; 0];
P = eye(6, 6);
P(1:3, 1:3) = zeros(3,3);
Q = [5e-4*eye(3,3) zeros(3,3);
    zeros(3,3) 0.0001*eye(3,3)];
transVecImu = zeros(1,3);
angVecImu = zeros(1,3);
posList = [X(1:3)];

x = [0,0,0];
for i=50:1200
    disp(i);
    
    filteredCloud_1 = cloudFilter(traj{i}, distThreshold, "VLP16");
    [edgeIdx_1, labelCloud_1, smoothnessCloud_1] = edgeDetector(filteredCloud_1, c_edge, c_plane);
    %get IMU's timestamp
    t0 = time(i);
    nearest_index0 = interp1(imuTime, 1:length(imuTime), t0, 'nearest');
    
    filteredCloud_2 = cloudFilter(traj{i+1}, distThreshold, "VLP16");
    [edgeIdx_2, labelCloud_2, smoothnessCloud_2] = edgeDetector(filteredCloud_2, c_edge, c_plane);
    %get IMU's timestamp
    t1 = time(i+1);
    nearest_index1 = interp1(imuTime, 1:length(imuTime), t1, 'nearest');
    
    if ~isnan(nearest_index0)
        [transVecImu, angVecImu] = imuAnalyser(imuTime, imuACC, imuGYR,...
            t0, t1, nearest_index0, nearest_index1);
    end
    
    
    
    
    %----------------------------------------------------------------------
    % evaluate the corespondence
    %----------------------------------------------------------------------
    
    
    
    
    % creating the edgeClouds
    
    edgeCloud_1 = select(filteredCloud_1, ~edgeIdx_1, 'OutputSize', 'full');
    edgeCloud_2 = select(filteredCloud_2, ~edgeIdx_2, 'OutputSize', 'full');
    
    % clustering the edge clouds
    
    [labelsEdge_1,numClusters_1] = pcsegdist(edgeCloud_1,distThreshold);
    badPtsIdx_1 = zeros(size1, size2);
    % select only the biggest edges
    for i=1:numClusters_1
        if nnz(labelsEdge_1==i)<minClusterSize
            [row, col] = find(labelsEdge_1==i);
            for j=1:size(row)
                badPtsIdx_1(row(j), col(j)) = 1;
            end
        end
    end
    edgeCloud_1 = select(edgeCloud_1, ~badPtsIdx_1, 'OutputSize', 'full');
    [labelsEdge_1,numClusters_1] = pcsegdist(edgeCloud_1,distThreshold);
    % generate an array of subclouds that represent an edge
    edgePoints_1 = {};
    for i=1:numClusters_1
        edgePoints = select(edgeCloud_1, find(labelsEdge_1==i)).Location;
        edgePoints_1{i} = edgePoints;
    end
    
    [labelsEdge_2,numClusters_2] = pcsegdist(edgeCloud_2,distThreshold);
    badPtsIdx_2 = zeros(size1, size2);
    % select only the biggest edges
    for i=1:numClusters_2
        if nnz(labelsEdge_2==i)<minClusterSize
            [row, col] = find(labelsEdge_2==i);
            for j=1:size(row)
                badPtsIdx_2(row(j), col(j)) = 1;
            end
        end
    end
    edgeCloud_2 = select(edgeCloud_2, ~badPtsIdx_2, 'OutputSize', 'full');
    [labelsEdge_2,numClusters_2] = pcsegdist(edgeCloud_2,distThreshold);
    % generate an array of subclouds that represent an edge
    edgePoints_2 = {};
    for i=1:numClusters_2
        edgePoints = select(edgeCloud_2, find(labelsEdge_2==i)).Location;
        edgePoints_2{i} = edgePoints;
    end
    
    % create the barycenter maps and centered edge clouds
    
    centeredEdgePoints_1 = {};
    centeredEdgePoints_2 = {};
    barycenterMap_1 = zeros(numClusters_1, 3);
    for i=1:numClusters_1
        barycenterMap_1(i,:) = barycenter(edgePoints_1{i});
        centeredEdgePoints_1{i} = edgePoints_1{i}-barycenterMap_1(i,:);
    end
    barycenterMap_2 = zeros(numClusters_2, 3);
    for i=1:numClusters_2
        barycenterMap_2(i,:) = barycenter(edgePoints_2{i});
        centeredEdgePoints_2{i} = edgePoints_2{i}-barycenterMap_2(i,:);
    end
    
    % match the subclouds
    
    corespondences = [];
    % prevent from double match
    idxList = [];
    for i=1:numClusters_1
        dist = 500;
        idx = 0;
        for j=1:numClusters_2
            try
                mahaldist = mean(mahal(centeredEdgePoints_1{i}, centeredEdgePoints_2{j}));
                barycenterDist = norm(barycenterMap_1(i,1:2)-barycenterMap_2(j,1:2));
                if mahaldist < dist && barycenterDist < barycenterThreshold
                    dist = mahaldist;
                    idx = j;
                end
            catch
                warning('dimension problem');
            end
        end
        if idx~=0 && ~ismember(idx, idxList) && mahaldist ~= 0
            idxList = [idxList, idx];
            corespondences = [corespondences; [i, idx]];
        end
    end
    
    
    
    
    
    %--------------------------------------------------------------------------
    % finding the correct rigid transform with Levenberg and Marquardt algorithm
    %--------------------------------------------------------------------------
    
    
    
    
    x0 = [0, 0, 0];
    %f = @(x)cost_mahalanobis(corespondences, barycenterMap_1, edgePoints_2, x);
    f = @(x)cost(corespondences, barycenterMap_1, barycenterMap_2, x);
    
    % remove outliers
    firstEval = f(x0);
    inliers = ~isoutlier(firstEval(:,1));
    corespondences = corespondences(inliers,:);
    
    %levenberg Marquardt optimisation
    f = @(x)cost(corespondences, barycenterMap_1, barycenterMap_2, x);
    lb = [-1.5, -1.5, -pi/3];
    ub = [1.5, 1.5, pi/3];
    try
        options = optimoptions('lsqnonlin','FunctionTolerance', 0.001);
        [x, resnorm] = lsqnonlin(f,x0,lb,ub,options);
        R = 1e-2.*eye(3, 3);
    catch
        warning('optimisation failure')
        R = 1000.*eye(3,3);
    end
    
    
    
    
    
    %----------------------------------------------------------------------
    % Fusing IMU with Kalman Filter
    %----------------------------------------------------------------------
    
    
    
    
    
    %parameters of EKF
    u = [transVecImu(1:2); angVecImu(3)];
    z = x(1:3)';
    
    %EKF
    [X, P] = kalmanFilter(X, P, u, z, Q, R);
    posList = [posList, X(1:3)];
    disp(X);
    
end

figure;
plot(posList(1,:), posList(2,:));