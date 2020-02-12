function c = c(ptCloud, coord, slice)
%C Compute the smoothness of a point

colmax = size(ptCloud.Location, 2);

%create S
S = zeros(1, slice, 3);
pointCounter = 1;
counter = 1;
while pointCounter<slice+1
    col = (coord(2)-slice/2)+counter-1;
    % handle the cases where col is over the limit or negative
    if col>colmax
        col = mod(col,colmax);
    elseif col<1
        col = col+colmax;
    end
    if col == 0
        break;
    end
    if ~isnan(ptCloud.Location(coord(1), col, 1)) && ptCloud.Location(coord(1), col, 1)~=0       
        S(1,pointCounter,:) =  ptCloud.Location(coord(1), col, :);
        pointCounter = pointCounter+1;
    end
    counter = counter+1;
end
%S = ptCloud.Location(coord(1), (coord(2)-size/2):(coord(2)+size/2), :);

normSum = 0;
pc_vect = [ptCloud.Location(coord(1),coord(2),1), ptCloud.Location(coord(1),coord(2),2),...
    ptCloud.Location(coord(1),coord(2),3)];
for i=1:slice
    S_vect = [S(1,i,1), S(1,i,2), S(1,i,3)];
    normSum = normSum + norm(S_vect-pc_vect);
end
c = (1/(counter+norm(pc_vect)))*normSum;
if c==0
    c = -1;
end
end
