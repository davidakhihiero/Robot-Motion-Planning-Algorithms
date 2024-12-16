function [] = plotGraph(graph, obstacle_vertices, r, fig, fig_title)
    figure(fig);
    title(fig_title);
    axis([0 1 0 1]); 
    %axis equal;
    axis square;
    hold on;

    for i = 1:size(obstacle_vertices, 1)
        xv = obstacle_vertices{i, 1};
        yv = obstacle_vertices{i, 2};

        fill(xv, yv, 'k');
        
    end
    plotGraphWithDubins(graph, r, fig);
end