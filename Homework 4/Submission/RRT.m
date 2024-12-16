function [rrt_graph, found_goal, path, cost_to_goal] = RRT(obstacles, x_init, x_goal, r, n_nodes, r_goal, seed, print_msg)
    if print_msg
        disp("Finding path with RRT (using Dubins curves)...");
    end

    rng(seed);

    rrt_graph = PGraph();
    found_goal = false;
    path = [];
    cost_to_goal = Inf;

    step_col_lin = 0.01; % step size for line collision checking
    step_col_ang = 0.01; % step size for curve collision checking
    step_steer = 4.1*r; % step size for steering

    collisionChecker = CollisionChecker(obstacles);
    
    % Line 1
    rrt_graph.add_node(x_init(1:2));
    rrt_graph.setvdata(rrt_graph.n, [0, 0, x_init(3)]);

    % Line 2
    for v = 1:n_nodes
        % Line 3
        x_rand = sampleFree(collisionChecker, 1);
        % Line 4
        x_nearest = Near(rrt_graph, x_rand, 0, r, true);
        x_nearest_data = rrt_graph.vdata(x_nearest);
     
        % Line 5
        x_new = steer([(rrt_graph.coord(x_nearest))', ...
            x_nearest_data(3)], x_rand, step_steer);

        % if bad_point % ignore points too close to x_nearest where dubins curve will fail
        %     % v = v - 1;
        %     continue;
        % end

        % Line 5b
  

        v1 = [rrt_graph.coord(x_nearest)' x_nearest_data(3)];
        v2 = x_new;

        %fprintf("r_from RRT = %f\n", r);
        [path_,path_length_] = shortestDubins([], v1, v2, r, ...
            collisionChecker, false, false, false);

        % Line 6
        if ~isempty(path_)
            % Line 7
            rrt_graph.add_node(x_new(1:2));


            rrt_graph.setvdata(rrt_graph.n, [x_nearest, x_nearest_data(2) + ...
                path_length_, x_new(3)]);
            
            rrt_graph.add_edge(x_nearest, rrt_graph.n);

            rrt_graph.setedata(rrt_graph.ne, path_);
        end
    end

    % Find closest node in goal region
    U = Near(rrt_graph, x_goal, r_goal, r, true);
    min_cost = Inf;
    x_goal_prime = 0;

    if isempty(U)
        return;
    end

    for u = U
        node_data= rrt_graph.vdata(u);
        if node_data(2) + norm(rrt_graph.coord(u) - x_goal(1:2)) < min_cost
            min_cost = node_data(2) + norm(rrt_graph.coord(u) - x_goal(1:2));
            x_goal_prime = u;
        end
    end

    found_goal = true;
    cost_to_goal = min_cost;
    x = x_goal_prime; % Start following the backpointers from the goal node...
    cont = 1;
    path(cont) = x;
    cont = cont + 1;
    vdata = rrt_graph.vdata(x);
    while vdata(1) ~= 0 % ... until it gets to start.
        node_data = rrt_graph.vdata(x);
        back_pt = node_data(1);
        x = back_pt;
        path(cont) = x;
        cont = cont + 1;
        vdata = rrt_graph.vdata(x);
    end

    path = flip(path);


end


