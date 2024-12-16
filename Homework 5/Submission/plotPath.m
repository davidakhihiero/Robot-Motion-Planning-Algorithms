function [p] = plotPath(fig, graph, nodes_along_path, style)
    p = [];
    figure(fig);
    hold on;
    for i = 1:length(nodes_along_path)-1
        V1 = graph.vertexlist(:, nodes_along_path(i));
        V2 = graph.vertexlist(:, nodes_along_path(i+1), :);

        p = plot([V1(1), V2(1)], [V1(2), V2(2)], style, 'LineWidth', 3);

    end
    %hold off;
end