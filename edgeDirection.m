function directions = edgeDirection(edgePoints)
directions = zeros(3, length(edgePoints));

for i=1:length(edgePoints)
    C = cov(edgePoints{i});
    [v,~] = eig(C);
    directions(:,i) = v(:,3);
end
end

