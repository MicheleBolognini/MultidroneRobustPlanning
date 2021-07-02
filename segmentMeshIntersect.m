function B = segmentMeshIntersect(points, mesh, precision)
% segmentMeshIntersect check if a mesh contains at least part of a segment.
% 
% B = segmentMeshIntersect(P,M)
% checks if the segment defined by P is at least partially contained in the mesh M.
% P is a 2-by-3 vector containing the two extremes in 3D as rows, M is a
% mesh object, with vertices and faces fields
% 
% ------
% Author: Michele Bolognini
% e-mail: michele.bolognini@polimi.it
% Created: 2020-10-29,    using Matlab R2019b

B = false;
% If the line containing the 2 points never intersects with the mesh, don't bother checking the middle
int = intersectLineMesh3d([points(1,:), points(2,:)-points(1,:)], mesh.vertices, mesh.faces);
int = unique(int,"rows");
if isempty(int)
    return
end
% If an even number of intersections is detected, then check  if each
% intersection is inside the made with the extremes of the 2 points
if mod(size(int,1),2) == 0
    limits = [min(points);
              max(points)]; % Vertices of the box
    for i = 1:size(int,1)
        if prod(int(i,:) >= limits(1,:)) && prod(int(i,:) <= limits(2,:))
            B = true;
            return
        end 
    end
    return
else
%     fprintf("IDK what to do with this:\n")
%     points
%     int
%     figure
%     displayMesh(mesh)
%     hold on
%     drawLine3d([points(1,:), points(2,:)-points(1,:)]);
%     scatter3(int(:,1), int(:,2), int(:,3), "*r")
    
end

if ~exist("precision","var")
    precision = 0.1;
end
vector = points(2,:) - points(1,:);
n = floor(norm(vector)/precision);
for i = 0:n % Check every "precision" meters if point is inside
    if inpolyhedron(mesh.faces, mesh.vertices, points(1,:) + vector*i/n)
        B = true;
        return
    end
end
if inpolyhedron(mesh.faces, mesh.vertices, points(2,:)) % Check if second point is inside
    B = true;
end
end