function F = newCost(corespondencesEdge, corespondencesPlane,...
    barycenterEdge_1, barycenterEdge_2,...
    edgeDirections_1, edgeDirections_2,normalsPlane_1, normalsPlane_2,...
    barycenterPlane_1, barycenterPlane_2, x)

F = zeros(length(corespondencesEdge)+length(corespondencesPlane), 2);
M = eul2rotm(x(4:6), 'XYZ');
H = makehgtform('translate', x(1:3),'xrotate', x(4), 'yrotate', x(5), 'zrotate', x(6));

for k=1:length(corespondencesEdge)
    i = corespondencesEdge(k,1);
    j = corespondencesEdge(k,2);
    R = [cos(x(6)) -sin(x(6));
        sin(x(6)) cos(x(6))];
    X1 = [barycenterEdge_1(i,1); barycenterEdge_1(i,2)];
    X2 = [barycenterEdge_2(j,1); barycenterEdge_2(j,2)];
    tX = [x(1); x(2)];
    delta_X = X2 - R\(X1-tX);
    F(k,1) = norm(delta_X);
    F(k,1) = 0;
    
    newDirection = M\edgeDirections_1(:,i);
    F(k,2) = dot(newDirection, edgeDirections_2(:,j))-norm(newDirection)*norm(edgeDirections_2(:,j));

end

for k=length(corespondencesEdge)+1:length(F)
    index = k-length(corespondencesEdge);
    i = corespondencesPlane(index,1);
    j = corespondencesPlane(index,2);
    
    newNormals_1 = M\normalsPlane_1(:,i);
    F(k,1) = dot(newNormals_1,normalsPlane_2(:,j))-norm(newNormals_1)*norm(normalsPlane_2(:,j));
    
    newBarycenter_1 = H\[barycenterPlane_1(i,:)';1];
    newBarycenter_1 = newBarycenter_1(1:3);
    d = -dot(normalsPlane_2(:,j), barycenterPlane_2(j,:)');
    newd = -dot(normalsPlane_2(:,j), newBarycenter_1);
    
    F(k,2) = abs(d-newd);
end

end

