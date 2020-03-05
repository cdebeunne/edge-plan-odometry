function F = cost_mahalanobis(corespondences, edgePoints_1, edgePoints_2, x)
%COST_MAHALANOBIS Summary of this function goes here
%   Detailed explanation goes here
F = zeros(size(corespondences,1),1);
for k=1:size(corespondences,1)
    i = corespondences(k,1);
    j = corespondences(k,2);
    R = [cos(x(3)) -sin(x(3));
        sin(x(3)) cos(x(3))];
    tX = [x(1); x(2)];
    edgeVector_1 = edgePoints_1{i}(:, 1:2)';
    edgeVector_2 = edgePoints_2{j}(:, 1:2)';
    newEdgeVector_1 = inv(R)*(edgeVector_1-tX);
    F(k,1) = mean(mahal(newEdgeVector_1', edgeVector_2'));
%    newBarycenter = barycenterMap(i,1:2);
%    rho = sqrt(barycenterMap(i,1)^2+barycenterMap(i,2)^2);
%    theta = atan2(barycenterMap(i,2), barycenterMap(i,1));
%    newBarycenter(1) = newBarycenter(1) - x(1) - rho*(cos(theta+x(3))-cos(theta));
%    newBarycenter(2) = newBarycenter(2) - x(2) - rho*(sin(theta+x(3))-sin(theta));
%    edgeSample = edgePoints{corespondences(k,2)};
%    F(i,1) = mahal(newBarycenter, edgeSample(:,1:2));
    
end
end

