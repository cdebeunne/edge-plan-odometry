function filteredCloud = cloudFilter(pc, type)
% Format the cloud and remove the ground 

% reshape point cloud

ptCloud = pointCloud(pc);
if type == "VLP16"
    ptCloud = pointcloudMatrixVLP16(ptCloud.Location);
elseif type == "HDL64"
    ptCloud = pointcloudMatrixHDL64(ptCloud.Location);
end

% remove the ground plane 

groundPtsIdx = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta',18);
filteredCloud = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');
end

