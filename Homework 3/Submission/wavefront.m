function [path, navigation_function, cost_to_goal] = wavefront(grid, q_start, q_goal, resolution, print_msg)
    if print_msg
        disp("Finding path with Wavefront...");
    end

    path = [];
    cost_to_goal = Inf;

    navigation_function = grid;
    goal_i = floor(1/resolution) - floor(q_goal(2) / resolution) + 1;
    goal_j = floor(q_goal(1) / resolution) + 1;

    navigation_function(goal_i, goal_j) = 2; % assign 2 to the goal cell

    navigation_function = BFS(navigation_function, [goal_i, goal_j], true);
    

    start_i = floor(1/resolution) - floor(q_start(2) / resolution) + 1;
    start_j = floor(q_start(1) / resolution) + 1;

    path = [path; start_j, start_i];
    goal_found = false;
    next = [start_i, start_j];

    while ~goal_found
        N = []; % Get the free neighbors
        if next(1) - 1 > 0 && navigation_function(next(1) - 1, next(2)) ~= 1
            N = [N [next(1)-1; next(2); navigation_function(next(1) - 1, next(2))]];
        end

        if next(1) + 1 < size(navigation_function, 1) && navigation_function(next(1) + 1, next(2)) ~= 1
            N = [N [next(1)+1; next(2); navigation_function(next(1) + 1, next(2))]];
        end

        if next(2) - 1 > 0 && navigation_function(next(1), next(2) - 1) ~= 1 
            N = [N [next(1); next(2)-1; navigation_function(next(1), next(2) - 1)]];
        end

        if next(2) + 1< size(navigation_function, 2)  && navigation_function(next(1), next(2) + 1) ~= 1 
            N = [N [next(1); next(2)+1; navigation_function(next(1), next(2) + 1)]];
        end
        
        if next(1) - 1 > 0 && next(2) - 1 > 0 && navigation_function(next(1) - 1, next(2) - 1) ~= 1
            N = [N [next(1)-1; next(2)-1; navigation_function(next(1) - 1, next(2) - 1)]];
        end

        if next(1) - 1 > 0 && next(2) + 1 > 0 && navigation_function(next(1) - 1, next(2) + 1) ~= 1
            N = [N [next(1)-1; next(2)+1; navigation_function(next(1) - 1, next(2) + 1)]];
        end

        if next(1) + 1 > 0 && next(2) - 1 > 0 && navigation_function(next(1) + 1, next(2) - 1) ~= 1
            N = [N [next(1)+1; next(2)-1; navigation_function(next(1) + 1, next(2) - 1)]];
        end

        if next(1) + 1 > 0 && next(2) + 1 > 0 && navigation_function(next(1) + 1, next(2) + 1) ~= 1
            N = [N [next(1)+1; next(2)+1; navigation_function(next(1) + 1, next(2) + 1)]];
        end

        [~, idx] = min(N(3, :)); % get the index of the neighbor with the smallest cost
        next = N(1:2, idx);
        next = next';
        path = [path; flip(next)];

        if next == [goal_i, goal_j]
            goal_found = true;
        end
    end

    cost_to_goal = size(path, 1) * resolution;
   
end