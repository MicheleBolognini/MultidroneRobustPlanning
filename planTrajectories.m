function solution = planTrajectories(points, params, mesh, mergedMesh)

%% Define some more parameters
params.fillerPointsDistance = 3; % distance of filler points along the vertex normal from which they are calculated
params.allPointsMinHeight = 1;   % minimum height above ground (z=0) for all points.
params.fillerPointBoundingBox = 4; % Width over maximum point coordinates over which non more filler points are considered close to a cluster
params.extendedGraphConnectionRadius = 8;   % maximum distance between connectable points in extended graph
params.minimalGraphConnectionRadius = 8;
startingPoint = params.startingPoint;

%% Find Clusters
numClusters = params.clusters;  % Predefined number of clusters
% K-Means Clustering. This is based on the geometric distribution of the
% points, does not account for obstacles
tic
clusterIdxs = kmeans(points, numClusters);
times.clustering = toc;
if params.debug
    figure(2)%, hold on, title("K-means Clustering Silhouette Index")
    %silhouette(points, clusterIdxs), grid on
    hold on, grid on, title("K-means Clustering")
    displayMesh(mesh)
    displayClustering(gca, points, clusterIdxs)
end
% Build cluster objects
clusters = cell(numClusters,1);
for c = 1:numClusters
    clusters{c}.id = c;
    clusters{c}.points = points(clusterIdxs==c,:);
    clusters{c}.centroid = mean(clusters{c}.points);
    clusters{c}.frontierIndexes = zeros(numClusters,1);
    clusters{c}.interClusterDist = zeros(numClusters,1);
    clusters{c}.interClusterPaths = {};
    clusters{c}.TSPsolution = [];
    clusters{c}.pathHome = [];
end
% Find filler points
normals = params.fillerPointsDistance * meshVertexNormals(mesh.vertices, mesh.faces);
fillerPoints = mesh.vertices + normals;
aux = fillerPoints(:,3) < params.allPointsMinHeight | inpolyhedron(mergedMesh, fillerPoints); % Filter out points that are either too low or inside the mesh
fillerPoints(aux,:) = [];
if params.debug
    figure(1)
    drawVector3d(mesh.vertices, normals);
    displayPoints(gca, fillerPoints, "*b");
end

%% For each cluster, find distance to neighboring clusters
eps = params.fillerPointBoundingBox;    % margin of box around points
for c = 1:numClusters
    clusters{c}.numPoints = size(clusters{c}.points,1);
    % Store all points, both of interest and filler, in a single vector
    clusters{c}.points = [clusters{c}.points; fillerPoints(...
        fillerPoints(:,1) > min(clusters{c}.points(:,1)) - eps & fillerPoints(:,1) < max(clusters{c}.points(:,1)) + eps & ...
        fillerPoints(:,2) > min(clusters{c}.points(:,2)) - eps & fillerPoints(:,2) < max(clusters{c}.points(:,2)) + eps & ...
        fillerPoints(:,3) > min(clusters{c}.points(:,3)) - eps & fillerPoints(:,3) < max(clusters{c}.points(:,3)) + eps, :)];
    clusters{c}.numAllPoints = size(clusters{c}.points,1);
    clusters{c}.A_extended = zeros(size(points,1));
    for i = 1:clusters{c}.numPoints
        for j=i+1:clusters{c}.numAllPoints
            dist = norm(clusters{c}.points(i,:) - clusters{c}.points(j,:));
            if dist <= params.extendedGraphConnectionRadius && ~segmentMeshIntersect([clusters{c}.points(i,:);  clusters{c}.points(j,:)], mergedMesh) 
                clusters{c}.A_extended(i,j) = dist;
                clusters{c}.A_extended(j,i) = dist;
            end
        end
    end
    clusters{c}.G_extended = graph(clusters{c}.A_extended);
    clusters{c}.A_minimal = zeros(clusters{c}.numPoints);
    clusters{c}.P_minimal = {}; % Store paths between nodes if there are intermediate ones
    p = 1;  % Path counter
    for i = 1:clusters{c}.numPoints-1
        for j = i+1:clusters{c}.numPoints
            dist = norm(clusters{c}.points(i,:) - clusters{c}.points(j,:));
            if dist <= params.minimalGraphConnectionRadius
                if segmentMeshIntersect([clusters{c}.points(i,:); clusters{c}.points(j,:)], mergedMesh)
                    [path, dist] = shortestpath(clusters{c}.G_extended,i,j);
                    clusters{c}.P_minimal{p,1} = path;
                    p = p+1;
                end
                clusters{c}.A_minimal(i,j) = dist;
                clusters{c}.A_minimal(j,i) = dist;
            end
        end
    end
end
%% Find neighboring points between clusters and their neighbors
% Add a series of points around the mesh to connect all clusters
outerFillerPoints = [];
% Scatter them along a cylinder around the structure
xmin = min(points(:,1));
xmax = max(points(:,1));
ymin = min(points(:,2));
ymax = max(points(:,2));
zmin = min(points(:,3));
zmax = max(points(:,3));
r = (max(xmax-xmin, ymax-ymin)+5)/2;
center = mean([xmax, ymax;
                xmin, ymin]);
for i = 5:5:zmax+5
    for theta = 0:45:360
    outerFillerPoints = [outerFillerPoints;
                        center+r*[cosd(theta) sind(theta)], i];
    end
end

for i = 1:numClusters-1
    for j = i+1:numClusters
        [idxs, dist] = minClusterDistance(clusters{i}, clusters{j}, mergedMesh);
        if dist == inf
            dist = 0;
        end
        clusters{i}.frontierIndexes(j) = idxs(1);
        clusters{i}.interClusterDist(j) = dist;
        clusters{j}.frontierIndexes(i) = idxs(2);
        clusters{j}.interClusterDist(i) = dist;
        if dist > 0
            clusters{i}.interClusterPaths{j} = [clusters{i}.points(idxs(1),:);
                                                clusters{j}.points(idxs(2),:)];
            clusters{j}.interClusterPaths{i} = clusters{i}.interClusterPaths{j}(end:-1:1);
        end
    end
end

% find path from each cluster to the starting point
A_aux = zeros(size(outerFillerPoints,1)+1);
for i = 1:size(A_aux,1)-1
    if i == 1
        p1 = startingPoint;
    else
        p1 = outerFillerPoints(i-1,:);
    end
    for j = i+1:size(A_aux,1)
        p2 = outerFillerPoints(j-1,:);
        if ~segmentMeshIntersect([p1;p2],mergedMesh)
            A_aux(i,j) = norm(p1-p2);
            A_aux(j,i) = A_aux(i,j);
        end
    end
end
startingPointGraph = graph(A_aux);
for i = 1:numClusters
    [path, dist] = minClusterStartingPointDistance(startingPointGraph, [startingPoint; outerFillerPoints], clusters{i}, mergedMesh);
    clusters{i}.frontierIndexes(i) = path(end);
    clusters{i}.interClusterDist(i) = dist;
    clusters{i}.pathHome = path;
end

% find path from each cluster to each non-adjacent cluster.
A_aux = A_aux(2:end, 2:end); % Only take the graph of filler points
max_AdjacencyGraph = graph(A_aux);

for i = 1:numClusters
    clusters{i}.outerFillerPointsDistances =  zeros(size(outerFillerPoints, 1),1);
    for k = 1:length(clusters{i}.outerFillerPointsDistances)
        if ~segmentMeshIntersect([outerFillerPoints(k,:);clusters{i}.points(1,:)],mergedMesh)
            clusters{i}.outerFillerPointsDistances(k) = norm(outerFillerPoints(k,:) - clusters{i}.points(1,:));
        end
    end
end


for i = 1:numClusters-1
%     Add node i to the graph
    A_aux = [A_aux, clusters{i}.outerFillerPointsDistances;
            clusters{i}.outerFillerPointsDistances', 0];
    for j = i+1:numClusters
        if clusters{i}.interClusterDist(j) == 0
%             Add node j to the graph
            A_aux = [A_aux, [clusters{j}.outerFillerPointsDistances; 0];
                    clusters{j}.outerFillerPointsDistances', 0, 0];
            G_aux = graph(A_aux);
            [path, dist] = shortestpath(G_aux, size(A_aux,1)-1, size(A_aux,1));
            clusters{i}.frontierIndexes(j) = 1;
            clusters{j}.frontierIndexes(i) = 1;
            clusters{i}.interClusterDist(j) = dist;
            clusters{j}.interClusterDist(i) = dist;
            clusters{i}.interClusterPaths{j} = [clusters{i}.points(1,:);
                                                outerFillerPoints(path(2:end-1), :);
                                                clusters{j}.points(1,:)];
            clusters{j}.interClusterPaths{i} = [clusters{j}.points(1,:);
                                                outerFillerPoints(path(end-1:-1:2), :);
                                                clusters{i}.points(1,:)];
%             Remove node j from the graph
            A_aux = A_aux(1:end-1, 1:end-1); 
        end
    end
%     Remove node i from the graph
    A_aux = A_aux(1:end-1, 1:end-1);
end

%% Limit connectivity by only keeping the params.graphMinDegree shortest connections
for i = 1:numClusters
    temp = clusters{i}.interClusterDist;
    temp(i) = 0;
    while nnz(temp) > params.graphMinDegree
%         Temporarily ignore distance to home
        [m, j] = max(temp); % Find max dist and its idx
        if nnz(clusters{j}.interClusterDist) > params.graphMinDegree % Don't cancel this if it would make the other nodes' degree < maximum degree
            clusters{i}.interClusterDist(j) = 0;
            clusters{i}.interClusterPaths{j} = [];
            clusters{j}.interClusterDist(i) = 0;
            clusters{j}.interClusterPaths{i} = [];
        end
        temp(j) = 0;
    end
end


%% Estimate each cluster's "weight" by solving a TSP between first and second frontier points
if params.debug
    figure(3)
    displayMesh(mesh)
    hold on
    grid on
end
for i = 1:numClusters
    for j= 1:length(clusters{i}.frontierIndexes)-1
        if j~=i && clusters{i}.frontierIndexes(j) ~= 0
            p1 = clusters{i}.frontierIndexes(j);
            break
        end
    end
    for k = j+1:length(clusters{i}.frontierIndexes)
        if k~=i && clusters{i}.frontierIndexes(k) ~= 0
            p2 = clusters{i}.frontierIndexes(k);
            break
        end
    end
    aux = zeros(length(clusters{i}.A_minimal),1);
    aux(p1) = 1;
    aux(p2) = 1;
    par.CostMatrixMulFactor = 1000;
    par.user_comment = "Looking for the shortest path on the local graph";
    par.vehicles = 1;
    par.infDist = 9999999;
    sol_temp = LKH_TSP([0, aux';            % Pass an enlarged matrix, adding one node at the beginning to find Hamiltonian Cycle...
                        aux, clusters{i}.A_minimal],...
                        par,strcat("LocalTSPInstance", num2str(i)),"LKH","problem_files");
    sol = sol_temp(2:end)-1;                    % Skip first node, as it is the fake one introduced earlier, and shift all nodes by one backwards
    clusters{i}.TSPsolution = sol;

    l = 0;
    if params.debug
        Gaux = graph(clusters{i}.A_minimal);
        plot(Gaux, "Xdata", clusters{i}.points(1:clusters{i}.numPoints,1), "Ydata", clusters{i}.points(1:clusters{i}.numPoints,2), "Zdata", clusters{i}.points(1:clusters{i}.numPoints,3))
    end
    for j = 1:length(sol)-1
        l = l + clusters{i}.A_minimal(sol(j), sol(j+1));
    end
    clusters{i}.weight = l;
end


% Plot
if params.debug
    figure(2)%, subplot(1,2,2)
    hold on
    for i = 1:numClusters-1
        for j = i+1:numClusters
            if size(clusters{i}.interClusterPaths{j},2) <= 2
                pts = [clusters{i}.points(clusters{i}.frontierIndexes(j),:); clusters{j}.points(clusters{j}.frontierIndexes(i),:)];
                scatter3(pts(:,1), pts(:,2), pts(:,3), 400, [0 0 0])
            else
                pts = [clusters{i}.interClusterPaths{j}];
                scatter3(pts(1,1), pts(1,2), pts(1,3), 400, [0 0 0])
                scatter3(pts(end,1), pts(end,2), pts(end,3), 400, [0 0 0])
            end
            plot3(pts(:,1), pts(:,2), pts(:,3), "Color", [0 0 0], "LineWidth", 1.5)
        end
    end
end

%% Compose the graph of clusters in the form of a Matrix
A_clusters = zeros(numClusters+1);    % This matrix describes the connections between clusters
centroids = zeros(numClusters+1,3);
centroids(1,:) = startingPoint;
weights = zeros(numClusters,1);
for c = 1:numClusters
    A_clusters(2:end,c+1) = clusters{c}.interClusterDist;
    centroids(c+1,:) = clusters{c}.centroid;
    A_clusters(c+1,c+1) = 0;
    A_clusters(1,c+1) = clusters{c}.interClusterDist(c);
    A_clusters(c+1,1) = A_clusters(1,c+1);
    weights(c) = clusters{c}.weight;
end
A_clusters(1,1) = 0;
A_clusters(A_clusters==inf)=0;  % Possible bug wheresome distances are sometimes inf

G_clusters = graph(A_clusters);
if params.debug
    figure
    plot(G_clusters, "XData", centroids(:,1), "YData", centroids(:,2), "ZData", centroids(:,3), "NodeLabel", round([0; weights]), "EdgeLabel", round(G_clusters.Edges.Weight));
    xlabel("X [m]"), ylabel("Y [m]"), zlabel("Z [m]"), grid on
    title("Relative position graph of clusters and their connections")
    hold on
end

vfprintf(params.verbose,"Now solving with %d clusters, %d drones\n", params.clusters, params.vehicles);
    
%% State and Solve CVRP
tic
l_max = max(G_clusters.Edges.Weight);
numv = params.vehicles;   % Number of vehicles
edges = [];
for i = 1:length(A_clusters)
    for j = 1:length(A_clusters)
        if A_clusters(i,j) ~= 0 && i ~= j
            edges = [edges; i j];
        end
    end
end
distances = zeros(1,size(edges,2));
for e = 1:length(edges)
    distances(e) = A_clusters(edges(e,1), edges(e,2));
    if edges(e,2) ~= 1
        distances(e) = distances(e) + clusters{edges(e,2)-1}.weight;
    end
end
numVars = length(edges)*numv;   % Number of optimization variables

A = [];
b = [];
Aeq = [];
beq = [];
nStops = length(A_clusters);

% Constraints:

% All nodes must be reached (and left) by exactly one vehicle, except for
% the root node, which is supposed to be the origin for all vehicles
for i = 1:nStops
    for j = 1:2
        idxs = edges(:,j) == i; % Indexes of nodes where stop i is origin/destination
        Aeq = [Aeq; repmat(idxs',1,numv)];
        if i == 1
            beq = [beq; numv]; % Node 1 must be left and take numv times
        else
            beq = [beq; 1];    % each other node must be left and taken 1 time
        end
    end
end

% If a vehicle visits a node, the same vehicle must leave it
for i = 1:nStops
    idxs1 = edges(:,1) == i;
    idxs2 = edges(:,2) == i;
    
    for k = 1:numv
        row = [];
        for j = 1:k-1
            row = [row, zeros(1, length(edges))];
        end
        row = [row, (idxs1-idxs2)'];
        for j = k:numv-1
            row = [row, zeros(1, length(edges))];
        end
        Aeq = [Aeq; row];
        beq = [beq; 0];
    end
    
end

% Each vehicle cannot span more tha params.vehicleCapacity - 2l_{max} meters
for k = 1:numv
    row = [];
    for j = 1:k-1
        row = [row, zeros(1, length(edges))];
    end
    row = [row, distances];
    for j = k:numv-1
        row = [row, zeros(1, length(edges))];
    end
    A = [A; row];
    b = [b; params.vehicleCapacity-2*l_max];
end

%For each vehicle, the sum of residual capacities of other drones must be
%less than params.vehicleCapacity
row = repmat(distances,1,numv);
A = [A; row];
b = [b; (numv-1)*params.vehicleCapacity - 2*numv*l_max];

opts = optimoptions('intlinprog', 'Display', 'off');

if params.minMaxing
    % Add a column for the threshold variable
    A = [A, zeros(size(A,1),1)];
    Aeq = [Aeq, zeros(size(Aeq,1),1)];

    % Threshold constraints
    for k = 1:numv
        row = [];
        for j = 1:k-1
            row = [row, zeros(1, length(edges))];
        end
        row = [row, distances];
        for j = k:numv-1
            row = [row, zeros(1, length(edges))];
        end
        A = [A; row, -1];
        b = [b; 0];
    end
     % Further setup for minmax formulation
    intcon = 1:numVars+1;
    lb = [zeros(numVars,1); 1];
    ub = [ones(numVars,1); inf];
    ctype = [repmat(['B'], 1, numv*length(distances)), 'I'];
    if strcmp(params.optimizer, "MATLAB")
        [x_CVRP_threshold, costopt, exitflag, output] = intlinprog([repmat(distances*0,1,numv), 1], intcon,A,b,Aeq,beq,lb,ub,opts);
    elseif strcmp(params.optimizer, "CPLEX")
        [x_CVRP_threshold, costopt, exitflag, output] = cplexmilp([repmat(distances*0,1,numv), 1], A, b, Aeq, beq, [], [], [], lb, ub, ctype);
    else
        error("ERROR: Unknown optimizer %s", params.optimizer)
    end
    x_CVRP = x_CVRP_threshold(1:end-1);
else
    % Setup for normal formulation
    intcon = 1:numVars;
    lb = [zeros(numVars,1)];
    ub = [ones(numVars,1)];
    if strcmp(params.optimizer, "MATLAB")
        [x_CVRP, costopt,exitflag,output] = intlinprog(repmat(distances,1,numv),intcon,A,b,Aeq,beq,lb,ub,opts);
    elseif strcmp(params.optimizer, "CPLEX")
        [x_CVRP, costopt, exitflag, output] = cplexbilp(repmat(distances,1,numv), A, b, Aeq, beq);
    else
        error("ERROR: Unknown optimizer %s", params.optimizer)
    end
end

if strcmp(params.optimizer, "CPLEX")
    iterations = output.iterations;
else
    iterations = 0;
end

x_CVRP_trips = logical(round(x_CVRP));
sol = reshape(x_CVRP_trips, [length(x_CVRP_trips)/numv,numv]);

if params.debug
    figure
    plot(G_clusters, "XData", centroids(:,1), "YData", centroids(:,2), "ZData", centroids(:,3), "LineStyle", "none");
    hold on
    for i = 1:size(sol,1)
        for j = 1:size(sol,2)
            if sol(i,j)==1
                pts = [centroids(edges(i,1),:); centroids(edges(i,2),:)];
                plot3(pts(:,1), pts(:,2), pts(:,3), "color", colors(j,:), "LineWidth", 1.5);
                hold on
            end
        end
    end
    grid on
end

%% Kill subtours and plot
% Kill subtours
takenEdges = logical(sum(sol,2));
G_sol = graph(edges(takenEdges,1), edges(takenEdges,2));
tourIdxs = conncomp(G_sol);
numtours = max(tourIdxs); % number of subtours
fprintf('CVRP solution found, # of subtours: %d\n',numtours);
while numtours > 1 % Repeat until there is just one subtour
    % Add the subtour constraints
    for ii = 1:numtours
        subTourIdx = find(tourIdxs == ii); % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        if ismember(1,subTourIdx)
            % If there is a tour comprising the origin keep it valid!
            continue
        end
        subTourEdges = zeros(length(edges),1);
        for m = 1:length(subTourIdx)-1
            for n = m+1:length(subTourIdx)
                edgeIndex1 = find(ismember(edges,[subTourIdx(m) subTourIdx(n)], "rows"));
                edgeIndex2 = find(ismember(edges,[subTourIdx(n) subTourIdx(m)], "rows"));
                subTourEdges(edgeIndex1) = 1;
                subTourEdges(edgeIndex2) = 1;
            end            
        end
         
        for k = 1:numv
            row = [];
            for j = 1:k-1
                row = [row, zeros(1, length(edges))];
            end
            row = [row, subTourEdges'];
            for j = k:numv-1
                row = [row, zeros(1, length(edges))];
            end
            if params.minMaxing
                A = [A; row 0];
            else
                A = [A; row];
            end
            b = [b; length(subTourIdx) - 1];
        end
    end

    % Try to optimize again
    if params.minMaxing
        if strcmp(params.optimizer, "MATLAB")
            [x_CVRP_threshold, costopt,exitflag,output] = intlinprog([repmat(distances*0,1,numv),1],intcon,A,b,Aeq,beq,lb,ub,opts);
        elseif strcmp(params.optimizer, "CPLEX")
            [x_CVRP_threshold, costopt, exitflag, output] = cplexmilp([repmat(distances,1,numv)*0, 1], A, b, Aeq, beq, [], [], [], lb, ub, ctype);
        end
        x_CVRP = x_CVRP_threshold(1:end-1);
    else
        if strcmp(params.optimizer, "MATLAB")
            [x_CVRP,costopt,exitflag,output] = intlinprog(repmat(distances,1,numv),intcon,A,b,Aeq,beq,lb,ub,opts);
        elseif strcmp(params.optimizer, "CPLEX")
            [x_CVRP, costopt, exitflag, output] = cplexbilp(repmat(distances,1,numv), A, b, Aeq, beq);
        end
    end
    if strcmp(params.optimizer, "CPLEX")
        iterations = iterations + output.iterations;
    end
    x_CVRP_trips = logical(round(x_CVRP));
    sol = reshape(x_CVRP_trips, [length(x_CVRP_trips)/numv,numv]);
    takenEdges = logical(sum(sol,2));
    G_sol = graph(edges(takenEdges,1), edges(takenEdges,2));
    
    % How many subtours this time?
    tourIdxs = conncomp(G_sol);
    numtours = max(tourIdxs); % number of subtours
    fprintf('CVRP solution found, # of subtours: %d\n', numtours)
end
times.CVRPsolution = toc;

[aux, x_CVRP_new] = calcSpans(x_CVRP_trips, numv, edges, distances);
sol = reshape(x_CVRP_new, [length(x_CVRP_trips)/numv,numv]);
spans = (sol'*distances')';

%% solve "real" TSPs
tic
finalEdges = edges(sum(sol,2) == 1,:);
for k = 1:numv
    loops{k} = finalEdges(k,:);
    next = finalEdges(k,2);
    while next ~= 1
        idx = find(finalEdges(:,1)==next);
        next = finalEdges(idx,2);
        loops{k} = [loops{k}, next];
    end    
end

for k = 1:numv
    if length(loops{k}) == 3
        c = loops{k}(2)-1;
        sol_temp = LKH_TSP(clusters{c}.A_minimal, par, strcat("LocalTSPInstance", num2str(c)), "LKH","problem_files");
        idx = find(sol_temp == clusters{c}.frontierIndexes(c));
        
        clusters{c}.TSPsolution = [ sol_temp(idx:end), sol_temp(1:idx-1)];
        clusters{c}.vehicleNumber = k;
    else
        for i = 2:length(loops{k})-1
            prev = loops{k}(i-1)-1;
            this = loops{k}(i)-1;
            next = loops{k}(i+1)-1;
            if prev == 0
                id1 = clusters{this}.frontierIndexes(this);
            else
                id1 = clusters{this}.frontierIndexes(prev);
            end
            if next == 0
                id2 = clusters{this}.frontierIndexes(this);
            else
                id2 = clusters{this}.frontierIndexes(next);
            end
            aux = zeros(length(clusters{this}.A_minimal),1);
            aux(id1) = 1;
            aux(id2) = 1;
            sol_temp = LKH_TSP([0, aux';
                        aux, clusters{this}.A_minimal],...
                        par,strcat("LocalTSPInstance", num2str(this)),"LKH","problem_files");
            sol_temp = sol_temp(2:end)-1;
            if sol_temp(1) == id2
                sol_temp = sol_temp(end:-1:1);
            end
            clusters{this}.TSPsolution = sol_temp;
            clusters{this}.vehicleNumber = k;
        end
    end
    
end
times.TSPsolution = toc;

% Compose the solution object
solution.vehicles = params.vehicles;
solution.trajectories = {};
for k = 1:numv
    c = loops{k}(2)-1;
    pts = [startingPoint; outerFillerPoints(clusters{c}.pathHome(2:end-1)-1,:)];
    pts = [pts; clusters{c}.points(clusters{c}.frontierIndexes(c),:)];
    for i = 2:length(loops{k})-1
        c = loops{k}(i)-1;
        new_points = clusters{c}.points(clusters{c}.TSPsolution,:);
        pts = [pts; new_points];
    end
    c = loops{k}(end-1)-1;
    pts = [pts; outerFillerPoints(clusters{c}.pathHome(end-1:-1:2)-1,:); startingPoint];
    solution.trajectories{k} = pts;
end
solution.centroids = centroids;
solution.outerFillerPoints = outerFillerPoints;
solution.times = times;
solution.spans = spans;
solution.clusterGraph = G_clusters;
solution.variables = sol;
solution.clusters = clusters;
solution.CVRPiterations = iterations;

delete *.log
end