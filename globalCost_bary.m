function Error = globalCost_bary(corespondencesEdge, corespondencesPlane,...
    edgeStruct_1, edgeStruct_2, ...
    planeStruct_1, planeStruct_2, x)
% the global cost function determining a 6 DOF rigid transform

%% Tranformation to optimize
T = x(1:3)';
R = eul2rotm(x(4:6), 'XYZ');


%% Deal with edge residuals
err_edge = zeros(size(corespondencesEdge,1),1);

for k=1:size(corespondencesEdge,1)
    i1 = corespondencesEdge(k,1); 
    i2 = corespondencesEdge(k,2);
    X1 = edgeStruct_1.barycenterMap(i1,1:2)';
    X2 = edgeStruct_2.barycenterMap(i2,1:2)';
    N1 = edgeStruct_1.directions(:,i1);
    N2 = edgeStruct_2.directions(:,i2);
    R2d = [cos(x(6)) -sin(x(6));
        sin(x(6)) cos(x(6))];

    X2pred = R2d'*(X1-T(1:2));
    N2pred = R'*N1;
    
    % L2 distance edge center
    dX = norm(X2pred-X2);
   
    % distance angulaire
    dang = dot(N2,N2pred)-(norm(N2)*norm(N2pred));
    err_edge(k) = dX;
end


%% Deal with planes residuals
err_planes = zeros(size(corespondencesPlane,1),1);

for k=1:size(corespondencesPlane,1)
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
    err_planes(k) = abs(dang);
    
end

Error = [err_edge; err_planes];