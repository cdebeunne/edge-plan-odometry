function  pos = groundtruth(posxyz ,theta)

% -------------------------------------------------------
%create a list of pose in relative coordinates
%--------------------------------------------------------

x = posxyz(:,1);
y = posxyz(:,2);
x = x(:)-x(1);
y = y(:)-y(1);
pos = [x, y];

R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
for k=1:size(pos,1)
    pos(k,:) = pos(k,:)*R;
end

end