load KITTI_SCAN_DAT.mat

% point cloud analysis parameters
c_edge = 0.2;
c_plane = 0.025;
distThreshold = 0.3;
minClusterSize = 10;
barycenterThreshold = 1.5;

% estimator parameters
xWorld = [0;0];
posList = [xWorld];
theta = 0;
R = eye(2,2);

for k=1:size(traj,2)
    disp(k);
    
    filteredCloud_1 = cloudFilter(traj{k}, distThreshold, "HDL64");
    [edgeIdx_1, labelCloud_1, smoothnessCloud_1] = edgeDetector(filteredCloud_1, c_edge, c_plane);
    
    filteredCloud_2 = cloudFilter(traj{k+1}, distThreshold, "HDL64");
    [edgeIdx_2, labelCloud_2, smoothnessCloud_2] = edgeDetector(filteredCloud_2, c_edge, c_plane);
    
    size1 = size(filteredCloud_1.Location, 1);
    size2 = size(filteredCloud_1.Location, 2);
    
    
    
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
                % handle strange cases of small clusters
                warning('dimension problem')
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
    catch
        warning('optimisation failure')
    end
    
    
    
    %----------------------------------------------------------------------
    % adding the new pose in world coordinates
    %----------------------------------------------------------------------
    
    
    
    theta = theta + x(3);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    dxWorld = R*x(1:2)';
    xWorld = xWorld + dxWorld;
    posList = [posList, xWorld];
    
end

load KITTI_groundtruth.mat

% display the results

plot(posList(1,:), posList(2,:));
hold on;
plot(x,y);
legend('Edge odometry', 'Groundtruth');