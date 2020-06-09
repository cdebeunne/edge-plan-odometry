% point cloud analysis parameters
c_edge = 0.08;
c_plane = 0.02;
distThresholdEdge = 0.2;
minClusterSizeEdge = 5;
barycenterThresholdEdge = 1.5;
distThresholdPlane = 0.3;
minClusterSizePlane = 50;
barycenterThresholdPlane = 3;

% creation of the point cloud

xyz = [[0,0,0]];
for k=1:100
    for i=1:100
        xcoord = 3 + 1*i/100;
        ycoord = -1;
        zcoord = -1 + k/100;
        xyz = [xyz; [xcoord, ycoord, zcoord]];
    end
end

for k=1:100
    for i=1:100
        ycoord = -0.5 + i/100;
        zcoord = -1 + k/100;
        xcoord = 5;
        xyz = [xyz; [xcoord, ycoord, zcoord]];
    end
end

for k = 1:10
    for i = 1:100
        ycoord = 4;
        xcoord = 0.5+ 3*k/10;
        zcoord = 0.5 - i/100;
        xyz = [xyz; [xcoord, ycoord, zcoord]];
    end
end

% creation of the rotated point cloud

xyz1 = [[0,0,0]];
R = eul2rotm([0,0,0.5], 'XYZ');
t = [1,0,0];
for i =1:length(xyz)
    xyz1 = [xyz1; (R\(xyz(i,:)-t)')'];
end
    
traj = {xyz, xyz1};

%--------------------------------------------------------------------------
% test of the algo
%--------------------------------------------------------------------------

% first filtering of the clouds
pc1 = pointcloudMatrixHDL64(xyz);
[edgeIdx_1, planeIdx_1, labelCloud_1, smoothnessCloud_1] = edgePlaneDetector(pc1.Location, c_edge, c_plane);

pc2 = pointcloudMatrixHDL64(xyz1);
[edgeIdx_2, planeIdx_2, labelCloud_2, smoothnessCloud_2] = edgePlaneDetector(pc2.Location, c_edge, c_plane);

figure(3)
pcshow(pc1.Location,labelCloud_1)
colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
title('Point Cloud planes and edges')

figure(4)
pcshow(pc2.Location,labelCloud_2)
colormap([[1 0 0]; [0 1 0]; [0 0 1]]);
title('Point Cloud planes and edges')

% creating the edgeClouds

edgeCloud_1 = select(pc1, ~edgeIdx_1, 'OutputSize', 'full');
edgeCloud_2 = select(pc2, ~edgeIdx_2, 'OutputSize', 'full');

% clustering the edge clouds

[edgePoints_1, centeredEdgePoints_1, barycenterEdge_1, directionsEdge_1, eigenEdge_1, labelsEdge_1, validEdge_1]...
    = clusteringEdge(edgeCloud_1, distThresholdEdge, minClusterSizeEdge);
[edgePoints_2, centeredEdgePoints_2, barycenterEdge_2, directionsEdge_2, eigenEdge_2, labelsEdge_2, validEdge_2]...
    = clusteringEdge(edgeCloud_2, distThresholdEdge, minClusterSizeEdge);

% creating the planeCloud 

planeCloud_1 = select(pc1, ~planeIdx_1, 'OutputSize', 'full');
planeCloud_2 = select(pc2, ~planeIdx_2, 'OutputSize', 'full');

% clustering the plane clouds

[planePoints_1, centeredPlane_1, barycenterPlane_1, normalsPlane_1,...
    normalsStd_1, normalsList_1, labelsPlane_1, validLabels_1]...
    = clusteringPlane(planeCloud_1, distThresholdPlane, minClusterSizePlane);
[planePoints_2, centeredPlane_2, barycenterPlane_2, normalsPlane_2,...
    normalsStd_2, normalsList_2, labelsPlane_2, validLabels_2]...
    = clusteringPlane(planeCloud_2, distThresholdPlane, minClusterSizePlane);

% figure(1);
% quiver3(planePoints_1{1}(:,1), planePoints_1{1}(:,2), planePoints_1{1}(:,3),...
%     normals_1{1}(:,1), normals_1{1}(:,2), normals_1{1}(:,3));
% 
% figure(2);
% quiver3(planePoints_2{1}(:,1), planePoints_2{1}(:,2), planePoints_2{1}(:,3),...
%     normals_2{1}(:,1), normals_2{1}(:,2), normals_2{1}(:,3));

corespondencesPlane = [1,1;2,2];
corespondencesEdge = [1,1;3,3;4,4;5,5;6,6;7,7;8,8;9,9;10,10;11,11;12,12;13,13;14,14];
planeWeights = ones(1,2);
edgeWeights = ones(1,14);
x0 = [0,0,0,0,0,0];
f = @(x)globalCost_orth(corespondencesEdge, corespondencesPlane,...
    edgePoints_1, directionsEdge_1, barycenterEdge_2, directionsEdge_2,...
    normalsPlane_1, normalsPlane_2, x);
[x, ~] = lsqnonlin(f,x0);
disp(x);

% x0 = [0,0,0];
% f = @(x)costEdge(corespondencesEdge, barycenterEdge_1, barycenterEdge_2, x);
% [x2d, ~] = lsqnonlin(f,x0);
% 
% x0 = [0,0,0,0];
% f = @(x)cost3d(corespondencesEdge, corespondencesPlane,...
%     edgeDirections_1, edgeDirections_2,normalsPlane_1, normalsPlane_2,...
%     barycenterPlane_1, barycenterPlane_2, x2d(1), x2d(2), x2d(3), x);
% [x3d, ~] = lsqnonlin(f,x0);
% disp([x2d(1:2), x3d(1:4)]);

% figure(1);
% pcshow(pc1);
% figure(2);
% pcshow(pc2);

