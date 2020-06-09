function sumDist = projPoint(barycenter, vector, pointArray)
% average distance of an array of point to an edge 

sumDist = 0;
vector = vector./norm(vector);

for i=1:length(pointArray)
    point = pointArray(i,:)';
    dot = (point-barycenter)'*vector;
    projPt = barycenter+dot.*vector;
    sumDist = sumDist + norm(point-projPt);
end
sumDist = sumDist/length(pointArray);
end

