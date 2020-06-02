function smoothnessCloud = createsmoothnessCloud(xyz, slice)
% create a smouthness cloud

row = size(xyz, 1);
column = size(xyz, 2);
smoothnessCloud = zeros(row, column);


%[r,c,v] = ind2sub(size(xyz),find(xyz >0));
[r,c] = find(abs(xyz(:,:,1)) > 0); % get only non NaN and >0

for r_idx = 1:size(xyz,1) % for each line of the amtrix
    
    % Find valid elements for this line
    %ir = find(r == r_idx);
    valid_col = c(r == r_idx)';
    if length(valid_col)<slice+1
        continue
    end

    % For each valid element, get slice+1 points (the central pt is included... may be removed ?)
    for n = 1:length(valid_col)
        
        % Get slice limits
        down = n-slice/2;
        up = n+slice/2;
        if down < 1
            down = length(valid_col)+down;
            S_idx = [1:up down:length(valid_col)];
        elseif up > length(valid_col)
            up = mod(up, length(valid_col));
            S_idx = [1:up down:length(valid_col)];
        else
            S_idx = down:up;
        end
        counter = valid_col(up)-valid_col(down)-slice;
        
        % Get neighbours coords
        S =  [xyz(r_idx, valid_col(S_idx),1); xyz(r_idx, valid_col(S_idx),2); xyz(r_idx, valid_col(S_idx),3)];


        % Process your c score
        coord = [r_idx, valid_col(n)];
        
        pc_vect = [xyz(coord(1),coord(2),1), xyz(coord(1),coord(2),2),...
        xyz(coord(1),coord(2),3)];
        g_vect = S - pc_vect';
        g_norm = vecnorm(g_vect,2,2);
        normSum = sum(g_norm);
        c_score = (1/(slice+norm(pc_vect)))*normSum;
        
        smoothnessCloud(r_idx, valid_col(n)) = c_score;
    end

end
