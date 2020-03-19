function c = c(xyz, coord, slice)
%C Compute the smoothness of a point
colmax = size(xyz, 2);

%create S
S = zeros(slice, 3);
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
    if ~isnan(xyz(coord(1), col, 1)) && xyz(coord(1), col, 1)~=0
        S(pointCounter,:) =  xyz(coord(1), col, :);
        pointCounter = pointCounter+1;
    end
    counter = counter+1;
end

pc_vect = [xyz(coord(1),coord(2),1), xyz(coord(1),coord(2),2),...
    xyz(coord(1),coord(2),3)];
g_vect = S - pc_vect;
g_norm = vecnorm(g_vect,2,2);
normSum = sum(g_norm);
c = (1/(counter+norm(pc_vect)))*normSum;
end

