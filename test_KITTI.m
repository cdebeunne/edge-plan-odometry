load KITTI_VEL_SCAN.mat

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

for k=1:size(traj,2)-1
    disp(k);
    if size(traj{k},1) == 0 || size(traj{k+1},1)==0
        dxWorld = R*dxWorld;
        xWorld = xWorld + dxWorld;
        posList = [posList, xWorld];
        continue
    end
    
    filteredCloud_1 = cloudFilter(traj{k},"HDL64");
    [edgeIdx_1, labelCloud_1, smoothnessCloud_1] = edgeDetector(filteredCloud_1.Location, c_edge, c_plane);
    
    filteredCloud_2 = cloudFilter(traj{k+1},"HDL64");
    [edgeIdx_2, labelCloud_2, smoothnessCloud_2] = edgeDetector(filteredCloud_2.Location, c_edge, c_plane);
    
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
    validLabels_1 = [];
    edgePoints_1 = {};
    % select only the biggest edges and generate the edgePoint array
    for i=1:numClusters_1
        if nnz(labelsEdge_1==i)>minClusterSize
            validLabels_1 = [validLabels_1, i];
            edgePoints = select(edgeCloud_1, find(labelsEdge_1==i));
            edgePoints_1{length(validLabels_1)} = edgePoints.Location;
        end
    end
    
    [labelsEdge_2,numClusters_2] = pcsegdist(edgeCloud_2,distThreshold);
    validLabels_2 = [];
    edgePoints_2 = {};
    % select only the biggest edges and generate the edgePoint array
    for i=1:numClusters_2
        if nnz(labelsEdge_2==i)>minClusterSize
            validLabels_2 = [validLabels_2, i];
            edgePoints = select(edgeCloud_2, find(labelsEdge_2==i));
            edgePoints_2{length(validLabels_2)} = edgePoints.Location;
        end
    end
    
    % create the barycenter maps and centered edge clouds
    
    centeredEdgePoints_1 = {};
    centeredEdgePoints_2 = {};
    barycenterMap_1 = zeros(length(validLabels_1), 3);
    for i=1:length(validLabels_1)
        barycenterMap_1(i,:) = barycenter(edgePoints_1{i});
        centeredEdgePoints_1{i} = edgePoints_1{i}-barycenterMap_1(i,:);
    end
    barycenterMap_2 = zeros(length(validLabels_2), 3);
    for i=1:length(validLabels_2)
        barycenterMap_2(i,:) = barycenter(edgePoints_2{i});
        centeredEdgePoints_2{i} = edgePoints_2{i}-barycenterMap_2(i,:);
    end
    
    % match the subclouds
    
    corespondences = [];
    % prevent from double match
    idxList = [];
    for i=1:length(validLabels_1)
        dist = 500;
        idx = 0;
        for j=1:length(validLabels_2)
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
    %f = @(x)cost_mahalanobis(corespondences, edgePoints_1, edgePoints_2, x);
    f = @(x)cost(corespondences, barycenterMap_1, barycenterMap_2, x);
    
    % remove outliers
    firstEval = f(x0);
    inliers = ~isoutlier(firstEval(:,1));
    corespondences = corespondences(inliers,:);
    
    %levenberg Marquardt optimisation
    %f = @(x)cost(corespondences, barycenterMap_1, barycenterMap_2, x);
    f = @(x)cost_mahalanobis(corespondences, edgePoints_1, edgePoints_2, x);
    lb = [-1.5, -1.5, -pi/3];
    ub = [1.5, 1.5, pi/3];
    try
        options = optimoptions('lsqnonlin','FunctionTolerance', 0.01);
        [x, resnorm] = lsqnonlin(f,x0,lb,ub,options);
    catch
        warning('optimisation failure')
    end
    
    
    
    %----------------------------------------------------------------------
    % adding the new pose in world coordinates
    %----------------------------------------------------------------------
    
    
    
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    theta = theta + x(3);
    dxWorld = R*x(1:2)';
    xWorld = xWorld + dxWorld;
    posList = [posList, xWorld];
    
end

load KITTI_OSTX.mat

% display the results

plot(posList(1,:), posList(2,:));
hold on;
plot(groundtruth(:,1),groundtruth(:,2));
legend('Edge odometry', 'Groundtruth');