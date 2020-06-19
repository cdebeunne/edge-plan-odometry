function Error = globalCost(corespondencesEdge, corespondencesPlane,...
    edgePoints_1, edgePoints_2,...
    planePoints_1, planePoints_2, x)
% the global cost function determining a 6 DOF rigid transform

%% Tranformation to optimize
T = x(1:3)';
R = eul2rotm(x(4:6), 'XYZ');

%% matching of the edges

err_Edge = zeros(length(corespondencesEdge), 1);
for k=1:size(corespondencesEdge, 1)
    i = corespondencesEdge(k,1);
    j = corespondencesEdge(k,2);
    predEdgePoints_1 = R\(edgePoints_1{i}'-T);
    
    err_Edge(k) = mean(mahal(predEdgePoints_1', edgePoints_2{j}));
end

%% matching of the planes 

err_Plane = zeros(size(corespondencesPlane,1), 1);
for k=1:size(corespondencesPlane, 1)
    i = corespondencesPlane(k,1);
    j = corespondencesPlane(k,2);
    predPlanePoints_1 = R\(planePoints_1{i}'-T);
    
    err_Plane(k) = mean(mahal(predPlanePoints_1', planePoints_2{j}));
    
end

Error = [err_Edge; err_Plane];