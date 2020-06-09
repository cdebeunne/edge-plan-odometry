function Error = globalCost_orth(corespondencesEdge, corespondencesPlane,...
    edgePoints_1, directionsEdge_1, barycenterEdge_2, directionsEdge_2,...
    normalsPlane_1, normalsPlane_2, x)
% the global cost function determining a 6 DOF rigid transform

%% Tranformation to optimize
T = x(1:3)';
R = eul2rotm(x(4:6), 'XYZ');

%% matching of the edges

errEdge = zeros(length(corespondencesEdge), 2);
for k=1:size(corespondencesEdge, 1)
    i = corespondencesEdge(k,1);
    j = corespondencesEdge(k,2);
    predEdgePoints_1 = R\(edgePoints_1{i}'-T);
    predDirectionsEdge_1 = R\directionsEdge_1(:,i);
    
    orthDist = projPoint(barycenterEdge_2(j,:)', directionsEdge_2(:,j), predEdgePoints_1');
    errEdge(k,1) = orthDist;
    
    dang = dot(predDirectionsEdge_1,directionsEdge_2(:,j))-(norm(predDirectionsEdge_1)*norm(directionsEdge_2(:,j)));
    errEdge(k,2) = 0;
end

%% matching of the planes 

errPlane = zeros(size(corespondencesPlane,1), 2);
for k=1:size(corespondencesPlane, 1)
    i = corespondencesPlane(k,1);
    j = corespondencesPlane(k,2);
    
    predNormals_1 = R\normalsPlane_1(:,i);
    dang = dot(predNormals_1,normalsPlane_2(:,j))-norm(predNormals_1)*norm(normalsPlane_2(:,j));
    errPlane(k,1) = abs(dang);
    errPlane(k,2) = 0;
end

Error = [errEdge; errPlane];

end

