function [] = plotGraph(graph, obstacle_vertices, fig)
    figure(fig);
    
    hold on;

    for i = 1:size(obstacle_vertices, 1)
        xv = obstacle_vertices{i, 1};
        yv = obstacle_vertices{i, 2};

        fill(xv, yv, 'k');
        
    end
    graph.plot();
end