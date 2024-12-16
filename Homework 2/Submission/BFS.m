function [found_goal, path, cost_to_goal] = BFS(graph, start_node, goal_node, print_msg)
    if print_msg
        disp("Finding path with BFS...");
    end
    found_goal = false;
    cost_to_goal = Inf;
    V = graph.n;                % Number of nodes
    E = graph.ne;               % Number of links (edges)
    
    O_size = V+E;           % Size of set O
    O = zeros(O_size, 3);   % Creation of set O 
    O_front = 1;            % Pointer to the front of Set O (FIFO)
    O_back = O_front;       % Pointer to the back of Set O
    
    C = zeros(V, 1);        % Set C
    
    control_O = zeros(V, 1);% Array used to control if a note enter Set O. Used to avoid repeated nodes in O
    
    v = start_node;                  % This is the starting Node. Could be any node
    O(O_back,:) = [v, v, 0];   % Adding node v and its backpointer to O
    O_back = mod(O_back, O_size)+1; % Increments the pointer to the back of O making sure we have a circular array.
    control_O(v) = 1;       % Indicates that v entered O. This will be used to prevent that v enters O again.
    
    while O_front ~= O_back % While O is not empty
        u = O(O_front,1);   % Remove u from O
        u_bkpt = O(O_front,2); % Backpointer of u
        u_cost = O(O_front, 3);
        O_front = mod(O_front, O_size)+1; % Increments the pointer
    
        if C(u) == 0        % Check if u is in C
            C(u) = u_bkpt;  % Add u to C. We are using the backpointer as a flag that u is in C.

            if u == goal_node
                found_goal = true;
                cost_to_goal = u_cost;
                break;
            end
    
            N = graph.neighbours(u); % Get the neighbors of u
            for n = N            % All neighbors of u...
                if C(n) == 0     % ... that are not in C ...
                    if control_O(n)==0  % ... and also are not in O (avoid repetitions in O) ...
                        cost_u_n = norm(graph.vertexlist(:,u) - graph.vertexlist(:,n));
                        O(O_back,:) = [n, u, u_cost + cost_u_n]; % ... are added to O with u as their backpointer.
                        O_back = mod(O_back, O_size)+1;
                        control_O(n) = 1;
                    end
      
                end
    
            end
        
        end
    
    end
    
    if found_goal
        x = goal_node; % Start following the backpointers from the goal node...
        cont = 1;
        path(cont) = x;
        cont = cont + 1;
        while x ~= start_node % ... until it gets to start.
            x = C(x);
            path(cont) = x;
            cont = cont + 1;
        end
        path = flip(path);
    else
        path = [];
    end

end