function [p] = plotPath(fig, graph, nodes_along_path, r, style)
    p = [];
    figure(fig);
    hold on;
    edges = graph.edgelist;

    for i = 1:length(nodes_along_path)-1
        n1 = nodes_along_path(i);
        V1 = graph.coord(n1);
        n1_data = graph.vdata(n1);

        V1 = [V1' n1_data(3)];

        n2 = nodes_along_path(i+1);
        V2 = graph.coord(n2);
        n2_data = graph.vdata(n2);

        V2 = [V2' n2_data(3)];

        condition = (edges(1, :) == n1) & (edges(2, :) == n2);
        e = find(condition);
        combination = graph.edata(e);

        % p = plot([V1(1), V2(1)], [V1(2), V2(2)], style, 'LineWidth', 3);
        [~, p] = dubinsPath(fig, V1, V2, r, ...
            combination, style, 3, false, true);

    end
    %hold off;
end