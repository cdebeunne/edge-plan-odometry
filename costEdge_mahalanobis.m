function errEdge = costEdge_mahalanobis(corespondences, edgePoints_1, edgePoints_2, x)
% an alternative cost edge function using mahalanobis distance

%% edge residuals
errEdge = zeros(size(corespondences,1),1);
for k=1:size(corespondences,1)
    i = corespondences(k,1);
    j = corespondences(k,2);
    R = [cos(x(3)) -sin(x(3));
        sin(x(3)) cos(x(3))];
    tX = [x(1); x(2)];
    edgeVector_1 = edgePoints_1{i}(:, 1:2)';
    edgeVector_2 = edgePoints_2{j}(:, 1:2)';
    newEdgeVector_1 = inv(R)*(edgeVector_1-tX);
    errEdge(k) = mean(mahal(newEdgeVector_1', edgeVector_2'));   
end
end

