function [searched_grid] = BFS(grid, goal_node, print_msg)
    if print_msg
        disp("Building navigation function with BFS...");
    end
  
    searched_grid = grid;
    V = size(grid, 1)^2;             % Number of nodes
    E = 4 * V;               % Number of links (edges)
    
    O_size = V+E;           % Size of set O
    O = zeros(O_size, 5);   % Creation of set O 
    O_front = 1;            % Pointer to the front of Set O (FIFO)
    O_back = O_front;       % Pointer to the back of Set O
    
    C = zeros(size(grid, 1), size(grid, 2), 2);        % Set C
    
    control_O = zeros(size(grid));% Array used to control if a note enter Set O. Used to avoid repeated nodes in O
    
    v = goal_node;                  % This is the goal Node
    O(O_back,:) = [v, v, 2];   % Adding node v and its backpointer to O
    O_back = mod(O_back, O_size)+1; % Increments the pointer to the back of O making sure we have a circular array.
    control_O(v(1), v(2)) = 1;       % Indicates that v entered O. This will be used to prevent that v enters O again.
    
    while O_front ~= O_back % While O is not empty
        u = O(O_front, 1:2);   % Remove u from O
        u_bkpt = O(O_front, 3:4); % Backpointer of u
        u_cost = O(O_front, 5);

        
     
        O_front = mod(O_front, O_size)+1; % Increments the pointer
    
        if C(u(1), u(2), 1) == 0       % Check if u is in C
            C(u(1), u(2), :) = u_bkpt;  % Add u to C. We are using the backpointer as a flag that u is in C.
            searched_grid(u(1), u(2)) = u_cost;
    
            N = []; % Get the free neighbors of u
            if u(1) - 1 > 0 && grid(u(1) - 1, u(2)) ~= 1
                N = [N [u(1)-1; u(2)]];
            end

            if u(1) + 1 < size(grid, 1) && grid(u(1) + 1, u(2)) ~= 1
                N = [N [u(1)+1; u(2)]];
            end

            if u(2) - 1 > 0 && grid(u(1), u(2) - 1) ~= 1 
                N = [N [u(1); u(2)-1]];
            end

            if u(2) + 1< size(grid, 2)  && grid(u(1), u(2) + 1) ~= 1 
                N = [N [u(1); u(2)+1]];
            end

            for n = N            % All neighbors of u...
                if C(n(1), n(2), 1) == 0     % ... that are not in C ...
                    if control_O(n(1), n(2))==0  % ... and also are not in O (avoid repetitions in O) ...
                        cost_u_n = 1;
                        O(O_back,:) = [n', u, u_cost + cost_u_n]; % ... are added to O with u as their backpointer.
                        O_back = mod(O_back, O_size)+1;
                        control_O(n(1), n(2)) = 1;
          
                    end
      
                end
    
            end
        
        end
    
    end

end