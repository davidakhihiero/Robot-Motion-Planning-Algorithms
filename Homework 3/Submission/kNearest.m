function N = kNearest(graph, v, k)
    % function to find the k nearest nodes of v
    [~, N] = graph.distances(graph.coord(v));
    N = N(2:k);
end