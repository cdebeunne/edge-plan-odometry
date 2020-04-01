function F = costPlane(corespondences, barycenterMap_1, barycenterMap_2, delta_x, delta_y, x)
%this is the cost function to find z, theta and phi
F = zeros(size(corespondences,1),2);
for k=1:size(corespondences,1)
    i = corespondences(k,1);
    j = corespondences(k,2);
    
    % roll calculation
    R_theta = [cos(x(2)) -sin(x(2));
        sin(x(2)) cos(x(2))];
    X1_theta = [barycenterMap_1(i,3); barycenterMap_1(i,2)];
    X2_theta = [barycenterMap_2(j,3); barycenterMap_2(j,2)];
    tX_theta = [x(1); delta_x];
    delta_theta = norm(X2_theta - R_theta\(X1_theta-tX_theta));
    
    % pitch calculation
    R_phi = [cos(x(3)) -sin(x(3));
        sin(x(3)) cos(x(3))];
    X1_phi = [barycenterMap_1(i,3); barycenterMap_1(i,1)];
    X2_phi = [barycenterMap_2(j,3); barycenterMap_2(j,1)];
    tX_phi = [x(1); delta_y];
    delta_phi = norm(X2_phi - R_phi\(X1_phi-tX_phi));

    F(k,1) = delta_theta;
    F(k,2) = delta_phi;
end
end
