function filteredCloud = cloudFilter(pc, type)
%Filter the cloud with segments

% denoise and reshape point cloud

ptCloud = pointCloud(pc);
if type == "VLP16"
    ptCloud = pointcloudMatrixVLP16(ptCloud.Location);
elseif type == "HDL64"
    ptCloud = pointcloudMatrixHDL64(ptCloud.Location);
end

% remove far points
% idx = (ptCloud.Location(:,:,1)>40) + (ptCloud.Location(:,:,1)<-40);
% idy = (ptCloud.Location(:,:,2)>40) + (ptCloud.Location(:,:,2)<-40);

% remove the ground plane 

groundPtsIdx = segmentGroundFromLidarData(ptCloud, 'ElevationAngleDelta',18);
filteredCloud = select(ptCloud, ~(groundPtsIdx), 'OutputSize', 'full');
end

