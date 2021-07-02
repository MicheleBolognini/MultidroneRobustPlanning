function [indexes, distance] = minClusterDistance(c1, c2, mesh)
% Given two clusters, calculates the minimum distance between pairs of
% points belonging to the clusters, returning the distance itself and the
% indexes of the two points. If the connection between two points
% intersects the mesh, it is ignored.
% [ID, dist] = minClustrDistance(C1, C2, M)
% C1, C2 are cluster objects with .points and .numPoints properties
% M is a mesh object with .vertices and .faces properties
% ID is a 2-by-1 vector containing the index of the point belonging to C1
% first and C2 second.
% dist is the euclidean distance between the points. dist is 0 if no
% direct connection is possible due to M.

distance = inf;
indexes = [0 0];
for i = 1:c1.numPoints
    for j = 1:c2.numPoints
        if norm(c1.points(i,:)-c2.points(j,:)) < distance && ~segmentMeshIntersect([c1.points(i,:); c2.points(j,:)], mesh)
            distance = norm(c1.points(i,:)-c2.points(j,:));
            indexes = [i, j];
        end
    end
end
if distance == inf distance = 0;
end