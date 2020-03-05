function pc = pointcloudMatrixHDL64(xyz)

pc = zeros(64, 4500, 3); % -24,9° à 2° // 0° à 360°

theta = (atan2(xyz(:,2), xyz(:,1))+pi)*180/pi;
phi = atan2(xyz(:,3), sqrt(xyz(:,2).^2 + xyz(:,1).^2))*180/pi+24.9;
xi = theta*size(pc,2)/360;
yi = phi*size(pc,1)/26.9;

for pt = 1:length(xi)
   rxi= max(round(xi(pt)), 1);
   ryi= max(round(yi(pt)), 1);
   if ryi < 65 && rxi < 4500
       pc(ryi, rxi, :) = xyz(pt, :);
   end
end

pc = pointCloud(pc);
end

