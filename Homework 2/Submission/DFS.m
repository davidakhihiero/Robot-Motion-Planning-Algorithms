function [found_goal, path] = DFS(graph, start_node, goal_node)
    
    found_goal = false;
    V = graph.n;                % Number of nodes
    E = graph.ne;               % Number of links (edges)
    
    O_size = V+E;           % Size of set O
    O = zeros(O_size, 2);   % Creation of set O 
    O_top = 1;            % Pointer to the top of Set O (LIFO/Stack)
    
    C = zeros(V, 1);        % Set C
    
    control_O = zeros(V, 1);% Array used to control if a note enter Set O. Used to avoid repeated nodes in O
    
    v = start_node;                  % This is the starting Node. Could be any node
    O(O_top,:) = [v, v];   % Adding node v and its backpointer to O 
    control_O(v) = 1;       % Indicates that v entered O. This will be used to prevent that v enters O again.
    
    while O_top ~= 0 % While O is not empty
        u = O(O_top,1);   % Remove u from O
        u_bkpt = O(O_top,2); % Backpointer of u
        O_top = O_top - 1;
    
        if C(u) == 0        % Check if u is in C
            C(u) = u_bkpt;  % Add u to C. We are using the backpointer as a flag that u is in C.

            if u == goal_node
                found_goal = true;
                break;
            end
    
            N = graph.neighbours(u); % Get the neighbors of u
            for n = N            % All neighbors of u...
                if C(n) == 0     % ... that are not in C ...
                    if control_O(n)==0  % ... and also are not in O (avoid repetitions in O) ...
                        O_top = mod(O_top, O_size)+1;
                        O(O_top,:) = [n, u]; % ... are added to O with u as their backpointer.
                        
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