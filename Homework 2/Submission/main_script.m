clc; clear; close all;

[graph, obstacle_vertices, collisionChecker] = createVisibilityGraph(false, true);

allow_invalid_nodes = true;

disp("Graph created")
fig = figure;

plotGraph(graph, obstacle_vertices, fig);
disp("Click a start position");

start_node = ginput(1);

% ensure node is valid
while (~collisionChecker.isFree(start_node) && ~allow_invalid_nodes)
    disp("Click a free space configuration!");
    start_node = ginput(1);
end

fprintf("Start position (%f, %f) clicked. Adding node to graph\n", start_node(1), start_node(2));
graph = addNode(graph, start_node, collisionChecker);
start_node_ID = graph.n;
clf(fig);
plotGraph(graph, obstacle_vertices, fig);
str = "q_{start}";
text(start_node(1) + 0.02, start_node(2), str, 'FontSize', 12, 'Color', 'r');

disp("Click a goal position");
goal_node = ginput(1);

% ensure node is valid
while (~collisionChecker.isFree(goal_node) && ~allow_invalid_nodes)
    disp("Click a free space configuration!");
    goal_node = ginput(1);
end

fprintf("Goal position (%f, %f) clicked. Adding node to graph\n", goal_node(1), goal_node(2));

graph = addNode(graph, goal_node, collisionChecker);
goal_node_ID = graph.n;
clf(fig);
plotGraph(graph, obstacle_vertices, fig);
str1 = "q_{start}";
text(start_node(1) + 0.02, start_node(2), str1, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text(goal_node(1) + 0.02, goal_node(2), str2, 'FontSize', 12, 'Color', 'r');

n = 100; %10000 is slow but was used for results
dt_bfs = 0;

for i = 1:n
    tic;
    [found_path_bfs, path_bfs, cost_bfs] = BFS(graph, start_node_ID, goal_node_ID, i==1);
    dt_bfs =  dt_bfs + toc;
end
dt_bfs =  dt_bfs / n;

dt_dijk = 0;
for i = 1:n
    tic;
    [found_path_dijk, path_dijk, cost_dijk] = Dijkstra(graph, start_node_ID, goal_node_ID, i==1);
    dt_dijk = dt_dijk + toc;
end
dt_dijk = dt_dijk / n;

dt_dijk2 = 0;
for i = 1:n
    tic;
    [found_path_dijk2, path_dijk2, cost_dijk2] = Dijkstra2(graph, start_node_ID, goal_node_ID, i==1);
    dt_dijk2 = dt_dijk2 + toc;
end
dt_dijk2 = dt_dijk2 / n;

dt_my_astar = 0;
for i = 1:n
    tic;
    [found_path_myastar, path_myastar, cost_myastar] = MyAstar(graph, start_node_ID, goal_node_ID, i==1);
    dt_my_astar = dt_my_astar + toc;
end
dt_my_astar = dt_my_astar / n;

dt_my_astar2 = 0;
for i = 1:n
    tic;
    [found_path_myastar2, path_myastar2, cost_myastar2] = MyAstar2(graph, start_node_ID, goal_node_ID, i==1);
    dt_my_astar2 = dt_my_astar2 + toc;
end
dt_my_astar2 = dt_my_astar2 / n;

dt_pgraph_astar = 0;
for i = 1:n
    tic;
    [path_astar, cost_astar] = graph.Astar(start_node_ID, goal_node_ID);
    dt_pgraph_astar = dt_pgraph_astar + toc;
end
dt_pgraph_astar = dt_pgraph_astar / n;


if found_path_bfs
    fprintf("A BFS path of length %f was found in %f seconds\n", cost_bfs, dt_bfs);
else
    disp("No BFS path was found");
end

if found_path_dijk
    fprintf("A Dijkstra path of length %f was found  in %f seconds\n", cost_dijk, dt_dijk);
else
    disp("No Dijkstra path was found");
end

if found_path_dijk2
    fprintf("A Dijkstra (using Binary Heap PQ) path of length %f was found  in %f seconds\n", cost_dijk2, dt_dijk2);
else
    disp("No Dijkstra path was found");
end

if found_path_myastar
    fprintf("An Astar path of length %f was found in %f seconds\n", cost_myastar, dt_my_astar);
else
    disp("No Astar path was found");
end

if found_path_myastar2
    fprintf("An Astar (using Binary Heap PQ) path of length %f was found in %f seconds\n", cost_myastar2, dt_my_astar2);
else
    disp("No Astar path was found");
end

if ~isempty(path_astar)
    fprintf("PGraph Astar path of length %f was found  in %f seconds\n", cost_astar, dt_pgraph_astar);
else
    disp("No PGraph Astar path was found");
end

hold on;
p_bfs = plotPath(fig, graph, path_bfs, 'r');
p_dijk = plotPath(fig, graph, path_dijk, 'g');
p_myastar = plotPath(fig, graph, path_myastar, 'b:');
p_astar = plotPath(fig, graph, path_astar, 'k--');
legend([p_bfs, p_dijk, p_myastar, p_astar], 'BFS', 'Dijkstra', 'My A*', 'PGraph A*');