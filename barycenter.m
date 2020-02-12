function b = barycenter(pc)
%Calculate the barycenter of a given pointCloud
b = [0, 0, 0];
for i=1:3
    b(i) = mean(pc(:,i));
end
end

