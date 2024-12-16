function graph = addNode(graph, node, collisionChecker)

    graph.add_node(node');
    node_id = graph.n;
    
    for i = 1:graph.n - 1
        v1 = node';
        v2 = graph.vertexlist(:,i);

        n_points = 100;
        points_on_line = discretizeLine(v1', v2', norm(v1-v2) / n_points);

        edge_valid = true;
        for j = 1:size(points_on_line, 1)
            point = points_on_line(j, :);
            if ~collisionChecker.isFree(point)
                edge_valid = false;
                break;
            end
        end

        if edge_valid
            graph.add_edge(i, node_id);
        end
   
    end
end