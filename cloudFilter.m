function filteredCloud = cloudFilter(pc, distThreshold, type)
%Filter the cloud with segments

% denoise and reshape point cloud

ptCloud = pcdenoise(pointCloud(pc));
if type == "VLP16"
    ptCloud = pointcloudMatrixVLP16(ptCloud.Location);
    badPtsIdx = zeros(16, 1800);
elseif type == "HDL64"
    ptCloud = pointcloudMatrixHDL64(ptCloud.Location);
    badPtsIdx = zeros(64, 4500);
end

% segment and remove the ground plane 

groundPtsIdx = segmentGroundFromLidarData(ptCloud);
ptCloudWithoutGround = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');
% [labels,numClusters] = pcsegdist(ptCloud,distThreshold);
% numClusters = numClusters+1;
% labels(groundPtsIdx) = numClusters;
% labelColorIndex = labels+1;
% 
% for i=1:max(max(labelColorIndex))
%     if nnz(labelColorIndex==i)<51
%         [row, col] = find(labelColorIndex==i);
%         for j=1:size(row)
%             badPtsIdx(row(j), col(j)) = 1;
%         end
%     end
% end
%filteredCloud = select(ptCloudWithoutGround, ~badPtsIdx, 'OutputSize', 'full');
filteredCloud = ptCloudWithoutGround;
end

