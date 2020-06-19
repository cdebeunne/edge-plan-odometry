function err_planes = costPlane(corespondencesPlane, normalsPlane_1, normalsPlane_2, barycenterPlane_1, barycenterPlane_2, x)
% this is the 6DOF cost function using planes only

%% Tranformation to optimize
T = x(1:3)';
R = eul2rotm(x(4:6), 'XYZ');

%% planes residuals
err_planes = zeros(size(corespondencesPlane,1),2);
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
    err_planes(k, :) = [0, abs(dang)]; 
end

end
