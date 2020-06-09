function Error = globalCost_damien(corespondencesEdge, corespondencesPlane,...
                                barycenterEdge_1, barycenterEdge_2, ...
                                directionsEdge_1, directionsEdge_2,...
                                normalsPlane_1, normalsPlane_2,...
                                barycenterPlane_1, barycenterPlane_2, x)
% the global cost function determining a 6 DOF rigid transform

%% Tranformation to optimize
T = x(1:3)';
R = eul2rotm(x(4:6), 'XYZ');


%% Deal with edge residuals
err_edge = zeros(2*length(corespondencesEdge),1);

for k=1:size(corespondencesEdge,1)
    i1 = corespondencesEdge(k,1); 
    i2 = corespondencesEdge(k,2);
    X1 = barycenterEdge_1(i1,1:2)';
    X2 = barycenterEdge_2(i2,1:2)';
    N1 = directionsEdge_1(:,i1);
    N2 = directionsEdge_2(:,i2);
    R2d = [cos(x(6)) -sin(x(6));
        sin(x(6)) cos(x(6))];

    X2pred = R2d'*(X1-T(1:2));
    N2pred = R'*N1;
    
    % L2 distance edge center
    dX = norm(X2pred-X2);
    % distance angulaire
    dang = dot(N2,N2pred)-(norm(N2)*norm(N2pred));
    err_edge(2*(k-1)+1:2*k) = [dX; abs(dang)];
end


%% Deal with planes residuals
err_planes = zeros(2*length(corespondencesPlane),1);

for k=1:size(corespondencesPlane,1)
    i1 = corespondencesPlane(k,1);
    i2 = corespondencesPlane(k,2);
    X1 = barycenterPlane_1(i1,:)';
    X2 = barycenterPlane_2(i2,:)';
    N1 = normalsPlane_1(:,i1);
    N2 = normalsPlane_2(:,i2);
    
    X2pred = R'*(X1-T);
    N2pred = R'*N1;
        
    % L2 distance de X2pred au plan 1 
    dX = N2'*(X2pred-X2);    
    
    % distance angulaire
    dang = dot(N2,N2pred)-(norm(N2)*norm(N2pred));
    err_planes(2*(k-1)+1:2*k) = [dX; abs(dang)];
    
end


Error = [err_edge; err_planes];