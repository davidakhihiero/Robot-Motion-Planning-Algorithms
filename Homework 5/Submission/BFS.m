function [] = BFS(graph, start_node, parent, print_msg)
    if print_msg
        disp("Rewiring with BFS...");
    end

    V = graph.n;                % Number of nodes
    E = graph.ne;               % Number of links (edges)
    
    O_size = V+E;           % Size of set O
    O = zeros(O_size, 1);   % Creation of set O 
    O_front = 1;            % Pointer to the front of Set O (FIFO)
    O_back = O_front;       % Pointer to the back of Set O
    
    C = zeros(V, 1);        % Set C
    C(parent) = 1; % Add the parent to C
    
    control_O = zeros(V, 1);% Array used to control if a note enter Set O. Used to avoid repeated nodes in O
    
    v = start_node;                  % This is the starting Node. Could be any node

    O(O_back,:) = v;   % Adding node v to O
    O_back = mod(O_back, O_size)+1; % Increments the pointer to the back of O making sure we have a circular array.
    control_O(v) = 1;       % Indicates that v entered O. This will be used to prevent that v enters O again.
    
    while O_front ~= O_back % While O is not empty
        %fprintf("O_front = %d, Oback = %d\n", O_front, O_back);
        
        u = O(O_front,1);   % Remove u from O

        u_data = graph.vdata(u);
        u_cost = u_data(2);
        O_front = mod(O_front, O_size)+1; % Increments the pointer
    
        if C(u) == 0        % Check if u is in C
            C(u) = 1;  % Add u to C. 

            % if u == goal_node
            %     found_goal = true;
            %     cost_to_goal = u_cost;
            %     break;
            % end
    
            N = graph.neighbours(u); % Get the neighbors of u
            for n = N            % All neighbors of u...
                n_data = graph.vdata(n);
                if C(n) == 0     % ... that are not in C ...
                    if control_O(n)==0  % ... and also are not in O (avoid repetitions in O) ...
                        cost_u_n = norm(graph.vertexlist(:,u) - graph.vertexlist(:,n));
                        O(O_back, 1) = n; % ... are added to O 
                        n_data(2) = u_cost + cost_u_n;
                        graph.setvdata(n, n_data);
                        O_back = mod(O_back, O_size)+1;
                        control_O(n) = 1;
                    end
      
                end
    
            end
        
        end
    
    end
   

end