function U = Near(graph, v, r)
    % function to find nodes within radius r of v
    [d, u] = graph.distances(v);
    U = u(find(d<=r));
    if ~isempty(U) && U(1) == 0
        U(1) = [];
    end
end