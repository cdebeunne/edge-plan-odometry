function errEdge = costEdge(corespondencesEdge,  barycenterMap_1, barycenterMap_2, x)
%this is the cost function using only edges to find dx, dy and dpsi


%% edge residuals
errEdge = zeros(length(corespondencesEdge), 2);
for k=1:size(corespondencesEdge, 1)
    i = corespondencesEdge(k,1);
    j = corespondencesEdge(k,2);
    R = [cos(x(3)) -sin(x(3));
        sin(x(3)) cos(x(3))];
    X1 = [barycenterMap_1(i,1); barycenterMap_1(i,2)];
    X2 = [barycenterMap_2(j,1); barycenterMap_2(j,2)];
    tX = [x(1); x(2)];
    delta_X = X2 - R\(X1-tX);
    errEdge(k,:) = delta_X';
end

end

