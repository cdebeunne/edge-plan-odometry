function Error = globalCost_orth(corespondencesEdge, corespondencesPlane,...
    edgeStruct_1, edgeStruct_2, ...
    planeStruct_1, planeStruct_2, x)
% the global cost function determining a 6 DOF rigid transform

%% Tranformation to optimize
T = x(1:3)';
R = eul2rotm(x(4:6), 'XYZ');

%% matching of the edges

errEdge = zeros(length(corespondencesEdge), 2);
for k=1:size(corespondencesEdge, 1)
    i1 = corespondencesEdge(k,1);
    i2 = corespondencesEdge(k,2);
    X2 = edgeStruct_2.barycenterMap(i2)';
    N2 = edgeStruct_2.directions(:,i2);
    
    predEdgePoints1 = R\(edgeStruct_1.edgePoints{i1}'-T);
    predN1 = R\N2;
    
    orthDist = projPoint(X2', N2, predEdgePoints1');
    errEdge(k,1) = orthDist;
    
    dang = dot(predN1,N2)-(norm(predN1)*norm(N2));
    errEdge(k,2) = 0;
end

%% matching of the planes 

errPlane = zeros(size(corespondencesPlane,1), 2);
for k=1:size(corespondencesPlane, 1)
    i1 = corespondencesPlane(k,1);
    i2 = corespondencesPlane(k,2);
    X1 = planeStruct_1.barycenterMap(i1,:)';
    X2 = planeStruct_2.barycenterMap(i2,:)';
    N1 = planeStruct_1.normalsPlane(:,i1);
    N2 = planeStruct_2.normalsPlane(:,i2);
    
    X2pred = R'*(X1-T);
    N2pred = R'*N1;
        
    % L2 distance de X2pred au plan 1 
    dX = N2'*(X2pred-X2);
    
    % distance angulaire
    dang = dot(N2,N2pred)-(norm(N2)*norm(N2pred));
    
    errPlane(k,1) = abs(dang);
    errPlane(k,2) = 0;
end

Error = [errEdge; errPlane];

end

