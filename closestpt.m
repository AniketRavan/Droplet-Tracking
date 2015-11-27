%finds the closest point to testpt in data
%data is an m x 2 matrix
function [pt] = closestpt(testpt,data)
    for i = 1:size(data,1)
        d(i) = pdist([testpt;data(i,:)],'euclidean');
    end
    m = min(d);
    idx = find(d == m);
    pt = data(idx,:);
end