function [graph, found_goal, path, cost_to_goal] = queryRoadmap(graph, obstacles, q_start, q_goal, algo, k, print_msg)
    if print_msg
        fprintf("Query phase for %s\n", algo);
    end

    found_goal = false;
    path = [];
    cost_to_goal = Inf;
    step = 0.01;

    collisionChecker = CollisionChecker(obstacles);

    nodes_to_add = [q_start, q_goal];
    q_start_ID = graph.n + 1;
    q_goal_ID = graph.n + 2;
    
    for q = nodes_to_add
        graph.add_node(q);
        Nq = kNearest(graph, graph.n, k);
        for q_prime = Nq
            if collisionFree(collisionChecker, q, graph.coord(q_prime), step)
                graph.add_edge(graph.n, q_prime);
                break;
            end
        end
    end

    [found_goal, path, cost_to_goal] = MyAstar(graph, q_start_ID, q_goal_ID, false);

end