function [] = plotGraphWithDubins(graph, r, fig)   
    figure(fig);
    hold on;

    % Plot all the edges and the nodes connecting edges
    for i = 1:graph.ne
        e = graph.edgelist(:, i);
        combination = graph.edata(i);
        v1_data = graph.vdata(e(1));
        v2_data = graph.vdata(e(2));
        v1 = [graph.coord(e(1)); v1_data(3)];
        v2 = [graph.coord(e(2)); v2_data(3)];

        plot(v1(1), v1(2), 'b.', MarkerSize=30);
        plot(v2(1), v2(2), 'b.', MarkerSize=30);

        dubinsPath(fig, v1, v2, r, combination, 'k', 1, false, true);
    end
  
end
