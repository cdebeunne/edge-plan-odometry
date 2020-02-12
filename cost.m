function F = cost(corespondences, barycenterMap_1, barycenterMap_2, x)
%this is the cost function to find the rigid transform
F = zeros(size(corespondences,1),2);
for k=1:size(corespondences,1)
    i = corespondences(k,1);
    j = corespondences(k,2);
    rho = sqrt(barycenterMap_1(i,1)^2+barycenterMap_1(i,2)^2);
    theta = atan2(barycenterMap_1(i,2), barycenterMap_1(i,1));
    delta_x = barycenterMap_2(j,1)-(barycenterMap_1(i,1)-x(1)-rho*(cos(theta+x(3))-cos(theta)));
    delta_y = barycenterMap_2(j,2)-(barycenterMap_1(i,2)-x(2)-rho*(sin(theta+x(3))-sin(theta)));
    F(k,1) = delta_x;
    F(k,2) = delta_y;
end
end

