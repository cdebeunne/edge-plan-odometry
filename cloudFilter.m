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

groundPtsIdx = segmentGroundFromLidarData(ptCloud);
filteredCloud = select(ptCloud, ~groundPtsIdx, 'OutputSize', 'full');
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
end

