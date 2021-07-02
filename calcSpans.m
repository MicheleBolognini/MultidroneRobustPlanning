function [spans, newX] = calcSpans(x_CVRP, vehicles, ed, distances)
%% WARNING, seems to return wrong span?
%% WARNING sometimes returns wrong size newX?
spans = zeros(1,vehicles);
n = length(ed);
x = zeros(n,1);
newX = zeros(size(x_CVRP));
for i = 1:vehicles
    x = x + x_CVRP(1+(i-1)*n:n*i);
end
takenEdges = ed(x==1,:);
bins = {};

k = 1;
for i = 1:length(takenEdges)    % Associate one arc exiting from node 1 to each vehicle
    if takenEdges(i,1) == 1
        bins{1,k} = takenEdges(i,:);
        spans(1,k) = distances(i);
        edgeIndex = find(ismember(ed, takenEdges(i,:), "rows"));
        newX(edgeIndex+(k-1)*n) = 1;
        k = k+1;
    end
end
for k = 1:vehicles
    next = bins{k}(2);
    while next ~= 1
        [aux, index] = ismember(next,takenEdges(:,1));
        bins{k} = [bins{k}; takenEdges(index,:)];
        next = takenEdges(index,2);
        spans(k) = spans(k) + distances(index);
        edgeIndex = find(ismember(ed, takenEdges(index,:), "rows"));
        newX(edgeIndex+(k-1)*n) = 1;
    end
end

end