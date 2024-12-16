function [found_goal, path, cost_to_goal] = MyAstar(graph, start_node, goal_node, print_msg)
    if print_msg
        disp("Finding path with A*...");
    end
    found_goal = false;
    cost_to_goal = Inf;
    V = graph.n;                % Number of nodes
    E = graph.ne;               % Number of links (edges)
    
    O_size = V+E;           % Size of set O
    O = Inf(O_size, 4);   % Creation of set O 
    O_front = 1;            % Pointer to the front of Set O (FIFO)
    O_back = O_front;       % Pointer to the back of Set O
    
    C = zeros(V, 2);        % Set C
   
    
    v = start_node;                  % This is the starting Node. Could be any node
    O(O_back,:) = [v, v, 0, 0];   % Adding node v and its backpointer to O
    O_back = mod(O_back, O_size)+1; % Increments the pointer to the back of O making sure we have a circular array.

    
    while O_front ~= O_back % While O is not empty
        u = O(O_front,1);   % Remove u from O
        u_bkpt = O(O_front,2); % Backpointer of u
        u_cost = O(O_front,3); % cost to u
        O_front = mod(O_front, O_size)+1; % Increments the pointer
    
        if C(u, 1) == 0        % Check if u is in C
            C(u, 1) = u_bkpt;  % Add u to C. We are using the backpointer as a flag that u is in C.
            C(u, 2) = u_cost;

            if u == goal_node
                found_goal = true;
                cost_to_goal = u_cost;
                break;
            end
    
            N = graph.neighbours(u); % Get the neighbors of u
            for n = N            % All neighbors of u...
                if C(n, 1) == 0     % ... that are not in C ...     
                    cost_u_n = norm(graph.vertexlist(:,u) - graph.vertexlist(:,n));
                    cost_n_goal = norm(graph.vertexlist(:,n) - graph.vertexlist(:,goal_node));
                    cost_start_n = u_cost + cost_u_n;
                    cost_start_goal = cost_start_n + cost_n_goal; % estimated cost to goal using euclidean distance heuristic

                                
                    O(O_back,:) = [n, u, cost_start_n, cost_start_goal]; % ... are added to O with u as their backpointer and cost
                    O = sortrows(O, 4);
                    O_back = mod(O_back, O_size)+1;
              
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