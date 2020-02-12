function pc = pointcloudMatrixVLP16(xyz)

pc = zeros(16, 1800, 3); % -15 à 15 / 0 à 360-

theta = (atan2(xyz(:,2), xyz(:,1))+pi)*180/pi;
phi = atan2(xyz(:,3), sqrt(xyz(:,2).^2 + xyz(:,1).^2))*180/pi +15;

xi = theta*size(pc,2)/360;
yi = phi*size(pc,1)/30;

for pt = 1:length(xi)
   ryi= max(round(xi(pt)), 1);
   rxi= max(round(yi(pt)), 1);
   pc(rxi, ryi, :) = xyz(pt, :);
end

pc = pointCloud(pc);