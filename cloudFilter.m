function filteredCloud = cloudFilter(pc, type)
% Format the cloud and remove the ground 

% reshape point cloud

ptCloud = pointCloud(pc);
if type == "VLP16"
    filteredCloud = pointcloudMatrixVLP16(ptCloud.Location);
elseif type == "HDL64"
    filteredCloud = pointcloudMatrixHDL64(ptCloud.Location);
end

end

