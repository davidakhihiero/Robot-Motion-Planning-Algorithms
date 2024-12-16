function prm_graph = PRMLearning(obstacles, n_nodes, r, seed, print_msg)
    if print_msg
        disp("Finding path with PRM...");
    end

    rng(seed);

    prm_graph = PGraph();
    step = 0.01; % step size for line collision checking

    collisionChecker = CollisionChecker(obstacles);
    % Algorithm 1: PRM (preprocessing phase)
    % Line 1
    

    % Line 2
    for v = 1:n_nodes
        % Line 3
        x_rand = sampleFree(collisionChecker, 1);
       
        % Line 5
        prm_graph.add_node(x_rand);

        % Line 4
        U = Near(prm_graph, prm_graph.coord(v), r);

        % Line 6
        for u = U
            % Line 7
            if ~DFS(prm_graph, v, u) % check if there is no path from x_rand to u
                % Line 8
                if collisionFree(collisionChecker, prm_graph.coord(v), prm_graph.coord(u), step)
                    prm_graph.add_edge(v, u);
                end
            end
        end

    end

end