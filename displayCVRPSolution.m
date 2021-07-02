function displayCVRPSolution(G_clusters, edges, centroids, solution, colors, fTitle)
%Displays the solution of the CVRP problem on the cluster graph G_clusters,
%with edges edges, centroids, solution, colors and figure  title
    plot(G_clusters, "XData", centroids(:,1), "YData", centroids(:,2), "ZData", centroids(:,3), "LineStyle", "none");
    hold on
    for i = 1:size(solution,1)
        for j = 1:size(solution,2)
            if solution(i,j)==1
                pts = [centroids(edges(i,1),:); centroids(edges(i,2),:)];
                plot3(pts(:,1), pts(:,2), pts(:,3), "color", colors(j,:), "LineWidth", 1.5);
                hold on
            end
        end
    end
    grid on
    title(fTitle)
end