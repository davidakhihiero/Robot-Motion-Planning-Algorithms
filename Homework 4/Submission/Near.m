function U = Near(graph, v, r_t, r, v_iscoord)
    % function to find nodes within radius r_t of v

    if ~v_iscoord
        v_data = graph.vdata(v);
        v1 = [graph.coord(v)' v_data(3)];
    else
        v1 = v;
    end

    U = [];

    for i = 1:graph.n
        if i ~= v      
            v2_data = graph.vdata(i);
            v2 = [graph.coord(i)' v2_data(3)];

            [~ ,d] = shortestDubins([], v1, v2, r, ...
                [], false, false, false);

            U = [U; d i];
        end
    end

    U = sortrows(U, 1);
    if r_t > 0
        U(U(:, 1) > r_t, :) = [];
        U = U(:,2);
    else
        U = U(1, 2);
    end

    U = U';

end