function [path, distance] = minClusterStartingPointDistance(G, points, cluster, mesh)
% Given a cluster, the starting point and a filler point cloud, calculates
% the minimum distance between the starting point and a point belonging to
% the cluster, returning a path on the out

distance = inf;
path = [1 0];
for i = 1:10:cluster.numPoints
    if norm(cluster.points(i,:)-points(1,:)) < distance
        if segmentMeshIntersect([cluster.points(i,:); points(1,:)], mesh)
            H = addnode(G,1);
            nodeIdx = H.numnodes;
            for j = 1:H.numnodes-1
                if ~segmentMeshIntersect([cluster.points(i,:); points(j,:)], mesh)
                    w = norm (cluster.points(i,:) - points(j,:));
                    H = addedge(H, j, nodeIdx, w);
                    H = addedge(H, nodeIdx, j, w);
                end
            end
            [p, d] = shortestpath(H, 1, nodeIdx);
            if d < distance
                distance = d;
                path = p;
                path(end) = i;
            end
        else
            distance = norm(cluster.points(i,:)-points(1,:));
            path = [1 i];
        end
    end
end
end