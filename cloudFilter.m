function filteredCloud = cloudFilter(pc, type)
%Filter the cloud with segments

% denoise and reshape point cloud

ptCloud = pointCloud(pc);
if type == "VLP16"
    ptCloud = pointcloudMatrixVLP16(ptCloud.Location);
elseif type == "HDL64"
    ptCloud = pointcloudMatrixHDL64(ptCloud.Location);
end

% remove the ground plane 

groundPtsIdx = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta',18);
filteredCloud = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');
[~, ~, outlierIndices] = pcfitplane(filteredCloud,0.5, [0,0,1]);
filteredCloud = select(filteredCloud, outlierIndices, 'OutputSize', 'full');
end

