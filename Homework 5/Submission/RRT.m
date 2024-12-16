function [rrt_graph, found_goal, path, cost_to_goal] = RRT(obstacles, x_init, x_goal, n_nodes, r_goal, ...
    time_to_improve_path, seed, print_msg)

    % Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. 
    % The international journal of robotics research, 30(7), 846-894.
    if print_msg
        disp("Finding path with RRT...");
    end

    rng(seed);

    rrt_graph = PGraph();
    found_goal = false;
    path = [];
    cost_to_goal = Inf;

    step_col = 0.01; % step size for line collision checking
    step_steer = 0.05; % step size for steering

    collisionChecker = CollisionChecker(obstacles);
    
    % Line 1
    rrt_graph.add_node(x_init);
    rrt_graph.setvdata(rrt_graph.n, [0, 0]);


    add_node = true;
    timer_started = false;

    % Line 2
    while (add_node || toc < time_to_improve_path) && rrt_graph.n < n_nodes
        % Line 3
        x_rand = sampleFree(collisionChecker, 1);
        % Line 4
        x_nearest = rrt_graph.closest(x_rand);
     
        % Line 5
        x_new = steer((rrt_graph.coord(x_nearest))', x_rand, step_steer);

        % Line 6
        if collisionFree(collisionChecker, rrt_graph.coord(x_nearest), x_new', step_col)
            % Line 7
            rrt_graph.add_node(x_new);
            x_nearest_data = rrt_graph.vdata(x_nearest);
            rrt_graph.setvdata(rrt_graph.n, [x_nearest, x_nearest_data(2) + ...
                norm(x_new - rrt_graph.coord(x_nearest)')]);
            
            rrt_graph.add_edge(x_nearest, rrt_graph.n);
        end

        if norm(x_new - x_goal) < r_goal
            add_node = false;
          
            if ~timer_started
                tic; % start timer
                timer_started = true;
            end
        end
    end

    % Find closest node in goal region
    U = Near(rrt_graph, x_goal, r_goal);
    min_cost = Inf;
    x_goal_prime = 0;

    if isempty(U)
        return;
    end

    for u = U
        node_data= rrt_graph.vdata(u);
        if node_data(2) + norm(rrt_graph.coord(u) - x_goal) < min_cost
            min_cost = node_data(2) + norm(rrt_graph.coord(u) - x_goal);
            x_goal_prime = u;
        end
    end

    found_goal = true;
    cost_to_goal = min_cost;
    x = x_goal_prime; % Start following the backpointers from the goal node...
    cont = 1;
    path(cont) = x;
    cont = cont + 1;
    while rrt_graph.vdata(x) ~= 0 % ... until it gets to start.
        node_data = rrt_graph.vdata(x);
        back_pt = node_data(1);
        x = back_pt;
        path(cont) = x;
        cont = cont + 1;
    end

    path = flip(path);


end


