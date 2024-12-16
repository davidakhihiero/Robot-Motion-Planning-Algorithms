function [found_goal, path, cost_to_goal] = Dijkstra2(graph, start_node, goal_node, print_msg)
    if print_msg
        disp("Finding path with Dijkstra (using Binary Heap PQ)...");
    end
    found_goal = false;
    cost_to_goal = Inf;
    V = graph.n;                % Number of nodes
    E = graph.ne;               % Number of links (edges)

    C = zeros(V, 2);        % Set C
  
    
    v = start_node;                  % This is the starting Node. Could be any node
   
    root_node = Node(0, v, v);
    PQ = BinaryHeap(root_node, V+E);
    
    while ~PQ.isEmpty() % While PQ is not empty
        [PQ, u] = PQ.getNextNodeInPQ();   % Remove u from PQ
        u_bkpt = u.back_pt; % Backpointer of u
        u_cost = u.cost; % cost to u
        % O_front = mod(O_front, O_size)+1; % Increments the pointer
    
        if C(u.idx, 1) == 0        % Check if u is in C
            C(u.idx, 1) = u_bkpt;  % Add u to C. We are using the backpointer as a flag that u is in C.
            C(u.idx, 2) = u_cost;

            if u.idx == goal_node
                found_goal = true;
                cost_to_goal = u_cost;
                break;
            end
    
            N = graph.neighbours(u.idx); % Get the neighbors of u
            for n = N            % All neighbors of u...
                if C(n, 1) == 0     % ... that are not in C ...
                    cost_u_n = norm(graph.vertexlist(:,u.idx) - graph.vertexlist(:,n));
                  
                    new_node = Node(u_cost + cost_u_n, n, u.idx);
                    PQ = PQ.insertNode(new_node);
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