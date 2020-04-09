function F = costPlane(corespondencesPlane, normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, x)
% this is the cost function using planes to find z, theta and phi

F = zeros(size(corespondencesPlane,1),2);
for k=1:size(corespondencesPlane,1)
    
    i = corespondencesPlane(k,1);
    j = corespondencesPlane(k,2);
    
    % tz evaluation
    F(k,1) = (barycenterPlane_1(i,3)-barycenterPlane_2(j,3))-x(3);
    
    % theta and psi evaluation
    eulang = [x(4), x(5), x(6)];
    R = eul2rotm(eulang, 'XYZ');
    newNormals_1 = R*normalsPlane_1(:,i);
    F(k,2) = dot(newNormals_1,normalsPlane_2(:,j))-norm(newNormals_1)*norm(normalsPlane_2(:,j));
    
end
end
