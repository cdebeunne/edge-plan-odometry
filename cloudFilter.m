function filteredCloud = cloudFilter(pc, type)
%Filter the cloud with segments

% denoise and reshape point cloud

ptCloud = pcdenoise(pointCloud(pc));
if type == "VLP16"
    ptCloud = pointcloudMatrixVLP16(ptCloud.Location);
elseif type == "HDL64"
    ptCloud = pointcloudMatrixHDL64(ptCloud.Location);
end

% segment and remove the ground plane 

groundPtsIdx = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta',18);
filteredCloud = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');
end

