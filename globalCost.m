function F = globalCost(corespondencesEdge, corespondencesPlane, barycenterEdge_1, barycenterEdge_2,...
    normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, edgeWeights, planeWeights, x)
% the global cost function determining a 6 DOF rigid transform

% matching of the edges

F = zeros(length(corespondencesEdge)+length(corespondencesPlane), 2);
for k=1:length(corespondencesEdge)
    i = corespondencesEdge(k,1);
    j = corespondencesEdge(k,2);
    R = [cos(x(6)) -sin(x(6));
        sin(x(6)) cos(x(6))];
    X1 = [barycenterEdge_1(i,1); barycenterEdge_1(i,2)];
    X2 = [barycenterEdge_2(j,1); barycenterEdge_2(j,2)];
    tX = [x(1); x(2)];
    delta_X = X2 - R\(X1-tX);
    F(k,1) = delta_X(1)*edgeWeights(k);
    F(k,2) = delta_X(2)*edgeWeights(k);
end

% matching of the planes 

for k=length(corespondencesEdge)+1:length(corespondencesEdge)+length(corespondencesPlane)
    index = k-length(corespondencesEdge);
    i = corespondencesPlane(index,1);
    j = corespondencesPlane(index,2);
    
    % tz evaluation
    F(k,1) = 0;
    
    % theta and psi evaluation
    eulang = [x(4), x(5), x(6)];
    R = eul2rotm(eulang, 'XYZ');
    newNormals_1 = R\normalsPlane_1(:,i);
    F(k,2) = dot(newNormals_1,normalsPlane_2(:,j))-norm(newNormals_1)*norm(normalsPlane_2(:,j));
    F(k,2) = F(k,2)*planeWeights(index);
end