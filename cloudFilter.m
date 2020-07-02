function filteredCloud = cloudFilter(pc, type)
% Format the cloud and remove the ground 

% reshape point cloud

if type == "VLP16"
    ptCloud = pointcloudMatrixVLP16(pc);
elseif type == "HDL64"
    ptCloud = pointcloudMatrixHDL64(pc);
end

groundPtsIdx = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta',18);
filteredCloud = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');

end

