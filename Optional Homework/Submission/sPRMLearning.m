function sprm_graph = sPRMLearning(obstacles, n_nodes, r, seed, print_msg)
    if print_msg
        disp("Finding path with sPRM...");
    end

    rng(seed);

    sprm_graph = PGraph();
    step = 0.01; % step size for line collision checking

    collisionChecker = CollisionChecker(obstacles);
    % Algorithm 1: sPRM (preprocessing phase)
    % Line 1
    for v = 1:n_nodes
        x_rand = sampleFree(collisionChecker, 1);
        sprm_graph.add_node(x_rand);
    end

    % Line 2
    for v = 1:sprm_graph.n
        % Line 3
        U = Near(sprm_graph, sprm_graph.coord(v), r);
        % Line 4
        for u = U
            % Line 5
            if collisionFree(collisionChecker, sprm_graph.coord(v), sprm_graph.coord(u), step)
                sprm_graph.add_edge(v, u);
            end
        end
    end

   

end