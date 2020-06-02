function labelCloud = classifyPlanesEdges(smoothnessCloud, c_edge, c_plane)
% create a label array determining which point is an edge or a plane

labelCloud = ones(size(smoothnessCloud,1), size(smoothnessCloud,2));

% Check if smoothness is good for edge
edgeCloud = (smoothnessCloud > c_edge);
% Get shift by one on left and right
dedgeCloud_left = [edgeCloud(2:end,:); edgeCloud(1, :)];
dedgeCloud_right = [edgeCloud(1:(end-1),:); edgeCloud(end, :)];
% Look if left OR rigth is also valid
edgeCloud = (edgeCloud & dedgeCloud_left) | (edgeCloud & dedgeCloud_right);

% Check if smoothness is good for plane
planeCloud = (smoothnessCloud < c_plane & ~edgeCloud);
% Get shift by one on left, right, up and down
dplaneCloud_left = [planeCloud(2:end,:); planeCloud(end,:)];
dplaneCloud_right = [planeCloud(end,:); planeCloud(1:(end-1),:)];
dplaneCloud_up = [planeCloud(:,2:end), planeCloud(:,1)];
dplaneCloud_down = [planeCloud(:,end), planeCloud(:,1:(end-1))];
% Look if shifted planes are also valid
planeCloud = (planeCloud & dplaneCloud_left) | (planeCloud & dplaneCloud_right)...
    | (planeCloud & dplaneCloud_up) | (planeCloud & dplaneCloud_down);

% Set the label : (1) undefied, (2) edge, (3) plane
labelCloud(edgeCloud) = 2;
labelCloud(planeCloud) = 3;

