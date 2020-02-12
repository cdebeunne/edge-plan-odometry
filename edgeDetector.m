function [edgeIdx, labelCloud, smoothnessCloud] = edgeDetector(pc, c_edge, c_plane)
%return arrays describing edges

% get size of pc
row = size(pc.Location, 1);
column = size(pc.Location, 2);

% create a smouthness cloud

smoothnessCloud = zeros(row, column);
for i=1:row
    for j=6:(column-6)
        if ~isnan(pc.Location(i,j,1)) && ~(pc.Location(i,j,1)==0)
            smoothnessCloud(i,j) = c(pc, [i,j], 10);
            if isnan(smoothnessCloud(i,j))
                smoothnessCloud(i,j) = -1;
            end
        else
            smoothnessCloud(i,j) = -1;
        end
    end
end

% classify planes or edges

labelCloud = ones(row, column);
for i=1:row
    for j=6:(column-6)
        if smoothnessCloud(i,j)>c_edge
            % here we define cases in order to prevent from isolated edge
            % points
            if i~=1 && i~=row
                if smoothnessCloud(i+1,j)>c_edge || smoothnessCloud(i-1,j)>c_edge
                    labelCloud(i,j) = 2;
                else
                    smoothnessCloud(i,j)=-1;
                end
            elseif i==1
                if smoothnessCloud(i+1,j)>c_edge
                    labelCloud(i,j) = 2;
                else
                    smoothnessCloud(i,j)=-1;
                end
            else
                if smoothnessCloud(i-1,j)>c_edge
                    labelCloud(i,j) = 2;
                else
                    smoothnessCloud(i,j)=-1;
                end
            end
        end
        if smoothnessCloud(i,j)<c_plane
            % here we define cases in order to prevent from isolated edge
            % points
            if i~=1 && i~=row
                if smoothnessCloud(i+1,j)<c_plane || smoothnessCloud(i-1,j)<c_plane
                    labelCloud(i,j) = 3;
                else
                    smoothnessCloud(i,j)=-1;
                end
            elseif i==1
                if smoothnessCloud(i+1,j)<c_plane
                    labelCloud(i,j) = 3;
                else
                    smoothnessCloud(i,j)=-1;
                end
            else
                if smoothnessCloud(i-1,j)<c_plane
                    labelCloud(i,j) = 3;
                else
                    smoothnessCloud(i,j)=-1;
                end
            end
        end
    end
end

% let's create the edge cloud

edgeIdx = ones(row, column);
for i=1:row
    for j=1:column
        if labelCloud(i,j)==2
            edgeIdx(i,j)=0;
        end
    end
end
end

