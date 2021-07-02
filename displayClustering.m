function displayClustering(ax, points, idxs, marker)
if ~exist("marker","var")
    marker = "filled";
end
colors = [0 0 0;
          0 0.447 0.741;
          0.85 0.325 0.098;
          0.929 0.694 0.125;
          0.494 0.184 0.556;
          0.466 0.674 0.188;
          0.301 0.745 0.933;
          0.635 0.075 0.184;
          1 0 0;
          0 1 0;
          0 0 1;
          0 1 1;
          1 0 1;
          1 1 0;
          1 1 1;
          0 0 0];
hold on
m = length(points);
for i = 1:m
    c = colors(mod(idxs(i),length(colors))+1,:);
    scatter3(ax, points(i,1),points(i,2),points(i,3), marker, "MarkerEdgeColor", c, "MarkerFaceColor", c);
end
end