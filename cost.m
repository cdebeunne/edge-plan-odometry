function F = cost(corespondences, barycenterMap_1, barycenterMap_2, x)
%this is the cost function to find the rigid transform
F = zeros(size(corespondences,1),2);
for k=1:size(corespondences,1)
    i = corespondences(k,1);
    j = corespondences(k,2);
    R = [cos(x(3)) -sin(x(3));
        sin(x(3)) cos(x(3))];
    X1 = [barycenterMap_1(i,1); barycenterMap_1(i,2)];
    X2 = [barycenterMap_2(j,1); barycenterMap_2(j,2)];
    tX = [x(1); x(2)];
    delta_X = X2 - R\(X1-tX);
    F(k,1) = delta_X(1);
    F(k,2) = delta_X(2);
end
end

