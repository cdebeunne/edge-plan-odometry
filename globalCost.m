function Error = globalCost(corespondencesEdge, corespondencesPlane, barycenterEdge_1, barycenterEdge_2,...
    normalsPlane_1, normalsPlane_2, x)
% the global cost function determining a 6 DOF rigid transform

%% matching of the edges

errEdge = zeros(length(corespondencesEdge), 2);
for k=1:size(corespondencesEdge, 1)
    i = corespondencesEdge(k,1);
    j = corespondencesEdge(k,2);
    R = [cos(x(6)) -sin(x(6));
        sin(x(6)) cos(x(6))];
    X1 = [barycenterEdge_1(i,1); barycenterEdge_1(i,2)];
    X2 = [barycenterEdge_2(j,1); barycenterEdge_2(j,2)];
    tX = [x(1); x(2)];
    delta_X = X2 - R\(X1-tX);
    errEdge(k,:) = delta_X;
end

%% matching of the planes 

errPlane = zeros(size(corespondencesPlane,1), 2);
for k=1:size(corespondencesPlane, 1)
    i = corespondencesPlane(k,1);
    j = corespondencesPlane(k,2);
    
    % tz evaluation
    errPlane(k,1) = 0;
    
    % theta and psi evaluation
    eulang = [x(4), x(5), x(6)];
    R = eul2rotm(eulang, 'XYZ');
    newNormals_1 = R\normalsPlane_1(:,i);
    errPlane(k,2) = dot(newNormals_1,normalsPlane_2(:,j))-norm(newNormals_1)*norm(normalsPlane_2(:,j));
end

Error = [errEdge; errPlane];