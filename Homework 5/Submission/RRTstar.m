function [rrt_graph, found_goal, path, cost_to_goal] = RRTstar(obstacles, x_init, x_goal, n_nodes, r, ...
    r_goal, time_to_improve_path, seed, print_msg)
    % Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. 
    % The international journal of robotics research, 30(7), 846-894.

    if print_msg
        disp("Finding path with RRT*...");
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
            X_near = Near(rrt_graph, x_new, r);
            
            % Line 8
            rrt_graph.add_node(x_new);
            

            x_nearest_data = rrt_graph.vdata(x_nearest);

            % Line 9
            x_min = x_nearest;
            c_min = x_nearest_data(2) + norm((rrt_graph.coord(x_nearest))' - x_new);
            
            % Line 10
            for x_near = X_near  
                x_near_data = rrt_graph.vdata(x_near);
                % Line 11
                if collisionFree(collisionChecker, rrt_graph.coord(x_near), x_new', step_col) && ...
                    (x_near_data(2) + norm((rrt_graph.coord(x_near))' - x_new)) < c_min
                    % Line 12
                    x_min = x_near;
                    c_min = x_near_data(2) + norm((rrt_graph.coord(x_near))' - x_new);
                end
            end
            % Line 13
            rrt_graph.add_edge(x_min, rrt_graph.n);
            rrt_graph.setvdata(rrt_graph.n, [x_min, c_min]);
         

            % Tree rewiring
            % Line 14
            for x_near = X_near
                x_near_data = rrt_graph.vdata(x_near);
                % Line 15
                if collisionFree(collisionChecker, x_new', rrt_graph.coord(x_near), step_col) && ...
                    (c_min + norm(x_new - (rrt_graph.coord(x_near))')) < x_near_data(2)
                    x_parent = x_near_data(1);
                    edges = rrt_graph.edgelist;
                    condition = (edges(1, :) == x_parent) & (edges(2, :) == x_near);
                    e = find(condition);
                    
                    % Line 16
                    x_near_data(1) = rrt_graph.n;
                    x_near_data(2) = c_min + norm(x_new - (rrt_graph.coord(x_near))');
                    rrt_graph.setvdata(x_near, x_near_data); % Update cost

                    rrt_graph.delete_edge(e);
                
                    rrt_graph.add_edge(rrt_graph.n, x_near);

                    % Update costs
                    BFS(rrt_graph, x_near, rrt_graph.n, false);
                    
                end
            end


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


