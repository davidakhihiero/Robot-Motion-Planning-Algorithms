function [rrt_graph, found_goal, path, cost_to_goal] = InformedRRTstar(obstacles, x_init, x_goal, ...
    N, r, r_goal, time_to_improve_path, seed, print_msg)
    
    % Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014, September). Informed RRT*: Optimal sampling-
    % based path planning focused via direct sampling of an admissible ellipsoidal heuristic. In 2014 IEEE/RSJ
    % International Conference on Intelligent Robots and Systems (pp. 2997-3004). IEEE.
    
    if print_msg
        disp("Finding path with informed RRT*...");
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

    % Line 3
    X_soln = Inf;

    add_node = true;
    timer_started = false; 

    % Line 5
    while (add_node || toc < time_to_improve_path) && rrt_graph.n < N
        % Line 6
        c_best = min(X_soln);

        % Line 7
        x_rand = informedSampleFree(collisionChecker, x_init, x_goal, c_best, 1);

        % Line 8
        x_nearest = rrt_graph.closest(x_rand);
     
        % Line 9
        x_new = steer((rrt_graph.coord(x_nearest))', x_rand, step_steer);

        % Line 10
        if collisionFree(collisionChecker, rrt_graph.coord(x_nearest), x_new', step_col)

            % Line 11
            rrt_graph.add_node(x_new);

            % Line 12
            X_near = Near(rrt_graph, x_new, r);
         
            x_nearest_data = rrt_graph.vdata(x_nearest);

            % Line 13
            x_min = x_nearest;
            % Line 14
            c_min = x_nearest_data(2) + norm((rrt_graph.coord(x_nearest))' - x_new);
            
            % Line 15
            for x_near = X_near  
                x_near_data = rrt_graph.vdata(x_near);

                %Line 16
                c_new = x_near_data(2) + norm((rrt_graph.coord(x_near))' - x_new);
                % Line 17
                if c_new < c_min
                    % Line 18
                    if collisionFree(collisionChecker, rrt_graph.coord(x_near), x_new', step_col)
                    
                        % Line 19
                        x_min = x_near;
                        % Line 20
                        c_min = c_new;
                    end
                end
            end
            % Line 21
            rrt_graph.add_edge(x_min, rrt_graph.n);
            rrt_graph.setvdata(rrt_graph.n, [x_min, c_min]);
         

            % Tree rewiring
            % Line 22
            for x_near = X_near
                x_near_data = rrt_graph.vdata(x_near);
                % Line 23
                c_near = x_near_data(2);
                % Line 24
                c_new = (c_min + norm(x_new - (rrt_graph.coord(x_near))'));
                % Line 25
                if c_new < c_near
                    % Line 26
                    if collisionFree(collisionChecker, x_new', rrt_graph.coord(x_near), step_col)
                        % Line 27
                        x_parent = x_near_data(1);
                        edges = rrt_graph.edgelist;
                        condition = (edges(1, :) == x_parent) & (edges(2, :) == x_near);
                        e = find(condition);
                        
                        
                        x_near_data(1) = rrt_graph.n;
                        x_near_data(2) = c_min + norm(x_new - (rrt_graph.coord(x_near))');
                        rrt_graph.setvdata(x_near, x_near_data); % Update cost
                        
                        % Line 28
                        rrt_graph.delete_edge(e);

                        % Line 29
                        rrt_graph.add_edge(rrt_graph.n, x_near);
    
                        % Update costs
                        BFS(rrt_graph, x_near, rrt_graph.n, false);
                    end
                    
                end
            end

            % Line 30
            if norm(x_new - x_goal) < r_goal
                % Line 31
                X_soln = [X_soln; c_min];
                add_node = false;
          
                if ~timer_started
                    tic; % start timer
                    timer_started = true;
                end
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


