function solution = failAndReplan(solution, params, mesh)
orderedEdges = {};
numv = params.vehicles;
edges = solution.clusterGraph.Edges.EndNodes;
edges = sortrows([edges; edges(:,2) edges(:,1)]);
distances = zeros(1,size(edges,1));
G_clusters = solution.clusterGraph;
centroids = solution.centroids;
clusters = solution.clusters;
startingPoint = params.startingPoint;
outerFillerPoints = solution.outerFillerPoints;
for i = 1:length(edges)
    [aux, idx] = ismember(edges(i,:), solution.clusterGraph.Edges.EndNodes, 'rows');
    if idx == 0
        [aux, idx] = ismember([edges(i,2) edges(i,1)], solution.clusterGraph.Edges.EndNodes, 'rows');
    end
    distances(i) = solution.clusterGraph.Edges.Weight(idx);
end

sol = solution.variables;
for i = 1:numv
    tempEdges = edges(sol(:,i)==1,:);
    
    idx = 2;
    while idx < size(tempEdges,1)
        to_find = tempEdges(idx-1,2);
        row_idx = find(tempEdges(:,1)==to_find);
        if row_idx ~= idx
            aux = tempEdges(idx,:);
            tempEdges(idx,:) = tempEdges(row_idx,:);
            tempEdges(row_idx,:) = aux;
        end
        idx = idx+1;
    end
    orderedEdges{1,i} = tempEdges;
end

%% Randomly select a drone and a point where it fails
failDrone = params.failDrone;
% It fails while inspecting the second node of the followng edge, i.e. does
% not visit it completely
failEdge = params.failEdge;
% Keep track of the last node each drone visits
lastVisitedNodes = zeros(numv,1);
for i = 1:numv
    if failEdge >= length(orderedEdges{i})
        lastVisitedNodes(i) = 1;
    else
        % The drone fails, we assume we finish the nodes the other ones are visiting before rerouting them
        if i == failDrone
            lastVisitedNodes(i) = orderedEdges{i}(failEdge,1);
        else
            lastVisitedNodes(i) = orderedEdges{i}(failEdge,2);
        end
    end
end
lastVisitedNodes(failDrone) = orderedEdges{failDrone}(failEdge,1);  % Last node actually visited by failed drone
fprintf("Drone %d failed after visiting node %d (at edge %d), replanning\n", failDrone, lastVisitedNodes(failDrone), failEdge)

%% Plot old solution, before failure
if params.debug
    figure
    subplot(2,2,1)
    displayCVRPSolution(G_clusters, edges, centroids, sol, colors, "Solution before Failure");
end

%% Calculate which nodes/edges have been traversed and their total length
traversedEdges = {}; % Edges that have been already traversed by the drones at the instant of failure.
traversedEdgesMask = zeros(size(edges,1),numv); % A mask representing those edges
traversedLengths = zeros(numv,1);
visitedNodes = [];
for i = 1:numv
    if failEdge >= size(orderedEdges{i},1)
        traversedEdges{i} = orderedEdges{i};
    else
        if i == failDrone
            traversedEdges{i} = orderedEdges{i}(1:failEdge-1,:);
        else
            traversedEdges{i} = orderedEdges{i}(1:failEdge,:);
        end
    end
    for j = 1:size(traversedEdges{i},1)
        idx = find(prod(edges == traversedEdges{i}(j,:),2));
        traversedEdgesMask(idx,i) = 1;
        traversedLengths(i) = traversedLengths(i) + distances(idx);
    end
    vNodes = unique(traversedEdges{i});
    visitedNodes(end+1:end+length(vNodes)-1) = vNodes(2:end); % Exclude the first node, 1, as it is always traversed.
end
% Keep the last visited nodes of non-failed drones as not visited, so that
% the new solution can include them (and start from there). Do keep the
% last one visited by the faled Drone
for i = 1:length(lastVisitedNodes)
    if lastVisitedNodes(i) ~= 1 && i ~= failDrone
        visitedNodes = visitedNodes(visitedNodes~=lastVisitedNodes(i));
    end
end


%% Draw the part of the solution that was already applied before the failure
if params.debug
    subplot(2,2,2)
    displayCVRPSolution(G_clusters, edges, centroids, traversedEdgesMask, colors, "State at failure");
    % Draw dotted line for failed drone
    pts = [centroids(lastVisitedNodes(failDrone),:); centroids(1,:)];
    plot3(pts(:,1), pts(:,2), pts(:,3), "k--","LineWidth", 1.5);
    subplot(2,2,3)
    displayCVRPSolution(G_clusters, edges, centroids, traversedEdgesMask, colors, "State at failure");
    pts = [centroids(lastVisitedNodes(failDrone),:); centroids(1,:)];
    plot3(pts(:,1), pts(:,2), pts(:,3), "k--","LineWidth", 1.5);
end

%% Represent the reduced version of the graph
G_clusters_reduced = rmnode(G_clusters,visitedNodes);
nonVisitedNodes = [];
for i = 1:params.clusters+1
    if ~ismember(i,visitedNodes)
        nonVisitedNodes(end+1) = i;
    end
end
centroids_reduced = centroids(nonVisitedNodes,:);
edgesToKeep = ones(size(edges,1),1);
for i =1:size(edges,1)
    if ismember(edges(i,1),visitedNodes) || ismember(edges(i,2),visitedNodes)
        edgesToKeep(i) = 0;
    end
end
edges_reduced = edges(edgesToKeep == 1,:);
distances_reduced = distances(:, edgesToKeep==1);
% Rename nodes in edges_reduced to match their new idxs
for i = 1:size(edges_reduced,1)
    [found, newLabel] = ismember(edges_reduced(i,1), nonVisitedNodes);
    edges_reduced(i,1) = newLabel;
    [found, newLabel] = ismember(edges_reduced(i,2), nonVisitedNodes);
    edges_reduced(i,2) = newLabel;
end
% Rename nodes in lastVisitedNodes_reduced for the same reason
lastVisitedNodes_reduced =  lastVisitedNodes;
for i = 1:size(lastVisitedNodes_reduced)
    [found, newLabel] = ismember(lastVisitedNodes_reduced(i), nonVisitedNodes);
    lastVisitedNodes_reduced(i) = newLabel;
end
%One of these will be 0, that related to the failed drone. Let's erase it
lastVisitedNodes_reduced(failDrone) = [];

%% Formulate new, smaller problem
% Update capacity limits
newCapacities = params.vehicleCapacity*ones(numv,1) - traversedLengths; % Updated capacities = previous - travelled
newCapacities(failDrone) = []; % the failed drone is removed
numv = numv-1;
tic

A = [];
b = [];
Aeq = [];
beq = [];
nStops = size(G_clusters_reduced.Nodes,1);
numVars = numv*length(edges_reduced);

% Constraints:



% All nodes must be reached (and left) by exactly one vehicle, except for
% the root node, which is supposed to be the origin for all vehicles
for i = 1:nStops
    for j = 1:2
        idxs = edges_reduced(:,j) == i; % Indices of nodes where stop i is origin/destination
        if i == 1
            % Impose maximum usage of drones
            A = [A; -repmat(idxs',1,numv)];
            b = [b; min(numv, nStops-1)];

            
            % Impose usage of all drones
%             Aeq = [Aeq; repmat(idxs',1,numv)];
%             beq = [beq; numv]; % Node 1 must be left and take numv times
        else
            Aeq = [Aeq; repmat(idxs',1,numv)];
            beq = [beq; 1];    % each other node must be left and taken 1 time
        end
    end
end

% If a vehicle visits a node, the same vehicle must leave it
for i = 1:nStops
    idxs1 = edges_reduced(:,1) == i;
    idxs2 = edges_reduced(:,2) == i;
    
    for k = 1:numv
        row = [];
        for j = 1:k-1
            row = [row, zeros(1, length(edges_reduced))];
        end
        row = [row, (idxs1-idxs2)'];
        for j = k:numv-1
            row = [row, zeros(1, length(edges_reduced))];
        end
        Aeq = [Aeq; row];
        beq = [beq; 0];
    end
end

% Each vehicle cannot span more tha params.vehicleCapacity meters
for k = 1:numv
    row = [];
    for j = 1:k-1
        row = [row, zeros(1, length(edges_reduced))];
    end
    row = [row, distances_reduced];
    for j = k:numv-1
        row = [row, zeros(1, length(edges_reduced))];
    end
    A = [A; row];
    b = [b; newCapacities(k)];
end

% If a non Failed drone is still out in a node, do as if it had just
% travelled to that node from the origin for free
for k = 1:numv
    if lastVisitedNodes_reduced(k) ~= 1
        %find index of the corresponding edge
        [found, index] = ismember([1, lastVisitedNodes_reduced(k)], edges_reduced, 'rows');
        distances_reduced(index) = 0; % Force its cost to 0
        % And force drone k to take it
        row = zeros(1, numVars);
        row((k-1)*length(edges_reduced) + index) = 1;
        Aeq = [Aeq;
               row];
        beq = [beq; 1];               
    end
end

if params.minMaxing
% Add a column for the threshold variable
    A = [A, zeros(size(A,1),1)];
    Aeq = [Aeq, zeros(size(Aeq,1),1)];

    % Threshold constraints
    for k = 1:numv
        row = [];
        for j = 1:k-1
            row = [row, zeros(1, length(edges_reduced))];
        end
        row = [row, distances_reduced];
        for j = k:numv-1
            row = [row, zeros(1, length(edges_reduced))];
        end
        A = [A; row, -1];
        b = [b; 0];
    end
    intcon = 1:numVars+1;
    lb = [zeros(numVars,1); 1];
    ub = [ones(numVars,1); inf];
    ctype = [repmat(['B'], 1, numv*length(distances_reduced)), 'I'];
    [x_CVRP_threshold, costopt, exitflag, output] = cplexmilp([repmat(distances_reduced,1,numv)*0, 1], A, b, Aeq, beq, [], [], [], lb, ub, ctype);
    x_CVRP = x_CVRP_threshold(1:end-1);
else
    intcon = 1:numVars;
    lb = [zeros(numVars,1)];
    ub = [ones(numVars,1)];
    opts = optimoptions('intlinprog', 'Display', 'off');
    [x_CVRP, costopt, exitflag, output] = cplexbilp(repmat(distances_reduced,1,numv), A, b, Aeq, beq);
end

iterations = output.iterations;

x_CVRP_trips = logical(round(x_CVRP));
sol = reshape(x_CVRP_trips, [length(x_CVRP_trips)/numv,numv]);

%% Kill subtours and plot
% Kill subtours
takenEdges = logical(sum(sol,2));
G_sol = graph(edges_reduced(takenEdges,1), edges_reduced(takenEdges,2));
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
        subTourEdges = zeros(length(edges_reduced),1);
        for m = 1:length(subTourIdx)-1
            for n = m+1:length(subTourIdx)
                edgeIndex1 = find(ismember(edges_reduced,[subTourIdx(m) subTourIdx(n)], "rows"));
                edgeIndex2 = find(ismember(edges_reduced,[subTourIdx(n) subTourIdx(m)], "rows"));
                subTourEdges(edgeIndex1) = 1;
                subTourEdges(edgeIndex2) = 1;
            end            
        end
         
        for k = 1:numv
            row = [];
            for j = 1:k-1
                row = [row, zeros(1, length(edges_reduced))];
            end
            row = [row, subTourEdges'];
            for j = k:numv-1
                row = [row, zeros(1, length(edges_reduced))];
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
        [x_CVRP_threshold, costopt, exitflag, output] = cplexmilp([repmat(distances_reduced,1,numv)*0, 1], A, b, Aeq, beq, [], [], [], lb, ub, ctype);
        x_CVRP = x_CVRP_threshold(1:end-1);
    else
        [x_CVRP, costopt, exitflag, output] = cplexbilp(repmat(distances_reduced,1,numv), A, b, Aeq, beq, x_CVRP);
    end
        
    iterations = iterations + output.iterations;
    x_CVRP_trips = logical(round(x_CVRP));
    sol = reshape(x_CVRP_trips, [length(x_CVRP_trips)/numv,numv]);
    takenEdges = logical(sum(sol,2));
    G_sol = graph(edges_reduced(takenEdges,1), edges_reduced(takenEdges,2));
    
    % How many subtours this time?
    tourIdxs = conncomp(G_sol);
    numtours = max(tourIdxs); % number of subtours
    fprintf('CVRP solution found, # of subtours: %d\n',numtours)
    
end

x_CVRP_new = x_CVRP_trips;
sol = reshape(x_CVRP_new, [length(x_CVRP_trips)/numv,numv]);
% reintroduce failed drone with no edges taken, for color correction
numv = numv+1;
aux = zeros(size(sol,1), numv);
curr = 1;
for i = 1:numv
    if i == failDrone
        aux(:,i) = zeros(size(sol,1),1);
    else
        aux(:,i) = sol(:,curr);
        curr = curr+1;
    end
end
sol = aux;
newSpans = (sol'*distances_reduced')';

%% Plot new solution on global graph
if params.debug
    displayCVRPSolution(G_clusters_reduced, edges_reduced, centroids_reduced, sol, colors,"Overlap");
    %figure
    subplot(2,2,4)
    displayCVRPSolution(G_clusters_reduced, edges_reduced, centroids_reduced, sol, colors,"Replanned Solution");
end
times.CVRPsolution = toc;


%% Obtain new solution in terms of old graph
newEdgesMask = zeros(size(traversedEdgesMask));
for k = 1:numv
    solution_edges = edges_reduced(sol(:,k)==1,:);
    for i = 1:size(solution_edges,1)
        solution_edges(i,1) = nonVisitedNodes(solution_edges(i,1));
        solution_edges(i,2) = nonVisitedNodes(solution_edges(i,2));
        if ~(solution_edges(i,1) == 1 && solution_edges(i,2) == lastVisitedNodes(k) ) % Do not count the edge if it was the fictitious one introduced to start from within the graph
            [found, idx] = ismember(solution_edges(i,:), edges,'rows');
            newEdgesMask(idx,k) = 1;
        end
    end
end
sol = traversedEdgesMask+newEdgesMask;
spans = (sol'*distances')';


%% solve "real" TSPs, only those you didn't solve already (= only new ones)
tic
for k = 1:numv
    finalEdges = edges(sol(:,k) == 1,:);
    loops{k} = [1];
    i = 1;
    while length(loops{k}) <= size(finalEdges,1)
        loops{k} = [loops{k}, finalEdges(i,2)];
        next = finalEdges(i,2);
        while next ~= 1
            idx = find(finalEdges(:,1)==next);
            next = finalEdges(idx,2);
            loops{k} = [loops{k}, next];
        end
        i = i+1;
    end
end
loops{failDrone} = [loops{failDrone}, 1];
%%TODO flip the order of loops to match the order at the beginning

for k = 1:numv
    if length(loops{k}) < 3 || k == failDrone % This should happen only if the drone fails immediately
        continue
    elseif length(loops{k}) == 3
        if failEdge >= 2    % If the node was already visited when the fail happened, skip
            continue
        end
        c = loops{k}(2)-1;
        sol_temp = LKH_TSP(clusters{c}.A_minimal, par, strcat("LocalTSPInstance", num2str(c)), "LKH-3.0.6","problem_files");
        idx = find(sol_temp == clusters{c}.frontierIndexes(c));
        
        clusters{c}.TSPsolution = [ sol_temp(idx:end), sol_temp(1:idx-1)];
        clusters{c}.vehicleNumber = k;
    else
        for i = 2:length(loops{k})-1
            prev = loops{k}(i-1)-1;
            this = loops{k}(i)-1;
            next = loops{k}(i+1)-1;
            % If the node is before the failed edge or it is the home node,
            % picked up at some point during the complete path, don't recalculate.
            if i <= failEdge+1 || this == 0
                continue
            end
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
            par.CostMatrixMulFactor = 1000;
            par.user_comment = "Looking for the shortest path on the local graph";
            par.vehicles = 1;
            par.infDist = 9999999;
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
% Display overall solution
if params.displayNewSolution
    colors = [0 0.447 0.741;
          0.85 0.325 0.098;
          0.929 0.694 0.125;
          0.494 0.184 0.556;
          0.466 0.674 0.188;
          0.301 0.745 0.933;
          0.635 0.075 0.184];
    figure
    hold on
    displayMesh(mesh)
    for k = 1:numv
%         For each vehicle, collect all the points it visits in order, then
%         plot lines through all of them
        if length(loops{k}) < 3
            continue
        end
        c = loops{k}(2)-1;
        pts = [startingPoint; outerFillerPoints(clusters{c}.pathHome(2:end-1)-1,:)];
        pts = [pts; clusters{c}.points(clusters{c}.frontierIndexes(c),:)];
        for i = 2:length(loops{k})-1
            if loops{k}(i) == 1
                new_points = [startingPoint];
            else
                c = loops{k}(i)-1;
                new_points = clusters{c}.points(clusters{c}.TSPsolution,:);
            end
            pts = [pts; new_points];
        end
        c = loops{k}(end-1)-1;
        pts = [pts; outerFillerPoints(clusters{c}.pathHome(end-1:-1:2)-1,:)];
        if k == failDrone
            pts = [pts; pts(end,1:2) 0];
            scatter3(pts(end,1), pts(end,2), 0, 200, 'x', "LineWidth", 4, "MarkerEdgeColor", "k")
            scatter3(pts(end,1), pts(end,2), 0, 80, colors(k,:), "filled")
        else
            pts = [pts; startingPoint];
        end
        plot3(pts(:,1), pts(:,2), pts(:,3), "color", colors(k,:), "LineWidth", 1.5)
    end
    grid on
end

% Compose the solution object
solution.vehicles = params.vehicles;
solution.trajectories = {};
for k = 1:numv
    c = loops{k}(2)-1;
    pts = [startingPoint; outerFillerPoints(clusters{c}.pathHome(2:end-1)-1,:)];
    pts = [pts; clusters{c}.points(clusters{c}.frontierIndexes(c),:)];
    for i = 2:length(loops{k})-1
        c = loops{k}(i)-1;
        if c == 0
            new_points = startingPoint;
        else
            new_points = clusters{c}.points(clusters{c}.TSPsolution,:);
        end
        pts = [pts; new_points];
    end
    c = loops{k}(end-1)-1;
    pts = [pts; outerFillerPoints(clusters{c}.pathHome(end-1:-1:2)-1,:); startingPoint];
    solution.trajectories{k} = pts;
end
solution.times = times;
solution.spans = spans;
solution.clusterGraph = G_clusters_reduced;
solution.variables = sol;
delete *.log
end