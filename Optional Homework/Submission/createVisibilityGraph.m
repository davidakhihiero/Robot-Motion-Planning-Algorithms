function [graph, obstacle_vertices, collisionChecker] = createVisibilityGraph(new_obstacles, dense)
    obstacles = createObstacles(new_obstacles, dense);
    collisionChecker = CollisionChecker(obstacles);
    
    graph = PGraph();
    obstacle_vertices = {};

    for i = 1:size(obstacles, 2)
        obstacle_vertices{i, 1} = obstacles{i}.Position(:, 1);
        obstacle_vertices{i, 2} = obstacles{i}.Position(:, 2);
    end

    % Add obstacles vertices and edges

    for i = 1:length(obstacles)
        obstacle = obstacles{i};
        for j = 1:size(obstacle.Position, 1)
            xv = obstacle.Position(j, 1);
            yv = obstacle.Position(j, 2);
            graph.add_node([xv; yv]); 
            % obstacle_vertices(i, j) = [xv; yv];
        end
    end

    %close all;

    % Add edges connecting all nodes to each other across the free space
    for i = 1:graph.n
        for j = i+1:graph.n
            v1 = graph.vertexlist(:,i);
            v2 = graph.vertexlist(:,j);

            step_size = 0.01;
            points_on_line = discretizeLine(v1', v2', step_size);

            edge_valid = true;
            for k = 1:size(points_on_line, 1)
                point = points_on_line(k, :);
                if ~collisionChecker.isFree(point)
                    edge_valid = false;
                    break;
                end
            end

            if edge_valid
                graph.add_edge(i, j);
            end
        end
    end

end