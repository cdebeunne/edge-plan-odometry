function [edgeIdx, planeIdx, labelCloud, smoothnessCloud] = edgePlaneDetector(xyz, c_edge, c_plane)
% return arrays describing edges and planes 

% create a smouthness cloud
smoothnessCloud = createsmoothnessCloud(xyz, 10);

% classify planes or edges
labelCloud = classifyPlanesEdges(smoothnessCloud, c_edge, c_plane);

% let's create the edge cloud
edgeIdx = ~(labelCloud == 2);
planeIdx = ~(labelCloud == 3);

end

