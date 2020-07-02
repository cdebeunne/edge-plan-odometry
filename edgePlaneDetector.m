function [edgeIdx, planeIdx, labelCloud, smoothnessCloud] = ...
    edgePlaneDetector(xyz, detector_params)
% return arrays describing edges and planes 

% create a smouthness cloud
smoothnessCloud = createsmoothnessCloud(xyz, 10);

% classify planes or edges
labelCloud = classifyPlanesEdges(smoothnessCloud,...
    detector_params.c_edge, detector_params.c_plane);

% let's create the edge cloud
edgeIdx = ~(labelCloud == 2);
planeIdx = ~(labelCloud == 3);

end

