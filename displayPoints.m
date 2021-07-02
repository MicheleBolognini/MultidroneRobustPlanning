function displayPoints(axis, points, color)
% displayPoints: a quick way of displaying a point cloud by only specifying
% them as a matrix.
%
% displayPoints(P,C). P is a N-by-3 column of 3D points, C is a string
% specifying color and marker type.
if ~exist("color","var")
    color = "r";
    scatter3(axis, points(:,1),points(:,2),points(:,3), "filled", "MarkerEdgeColor", color, "MarkerFaceColor", color)
else
    scatter3(axis, points(:,1),points(:,2),points(:,3), color)
end
end