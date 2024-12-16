clc; clear; close all;

[vis_graph, obstacle_vertices, collisionChecker] = createVisibilityGraph(false, true);

obstacles = createObstacles(false, true);
start_node = [0.01 0.01];
goal_node = [0.9 0.9];

%% sPRM 
n_nodes = 200;
r = 0.2;
seed = 7;
sprm_graph = sPRMLearning(obstacles, n_nodes, r, seed, true);

fig_sprm = figure;
figure(fig_sprm);
plotGraph(sprm_graph, obstacle_vertices, fig_sprm, "sPRM");

k = 5;
start = start_node';
% start = start';

goal = goal_node';

[sprm_graph, found_goal_sprm, path_sprm, cost_sprm] = ...
                                  queryRoadmap(sprm_graph, obstacles, start, goal, "sPRM", k, true);
plotGraph(sprm_graph, obstacle_vertices, fig_sprm, "sPRM");
str = "q_{start}";
text((start(1) + 0.02), (start(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((goal(1) + 0.02), (goal(2)), str2, 'FontSize', 12, 'Color', 'r');
hold on;
p_sprm = plotPath(fig_sprm, sprm_graph, path_sprm, 'y');


trajectory_sprm = start_node;
n = 5;
k = 4;
dt = 0.01;
T = 1;
v = 0; % initial velocity

for i = 1:size(path_sprm, 2)-1
    waypoint_start = sprm_graph.coord(path_sprm(i));
    waypoint_end = sprm_graph.coord(path_sprm(i+1));
    if i < size(path_sprm, 2)-1
        s = norm(waypoint_end - waypoint_start);
        a = 2*(s - v*T) / (T^2);
        velocity_at_end = v^2 + 2*a*s;
    else
        velocity_at_end = 0;
    end
    x_traj = getTrajectory(n, k, dt, T, waypoint_start, waypoint_end, ...
        [v, velocity_at_end], collisionChecker, true);
    y_traj = getTrajectory(n, k, dt, T, waypoint_start, waypoint_end, ...
        [v, velocity_at_end], collisionChecker, false);
    trajectory_sprm = [trajectory_sprm; x_traj, y_traj];
    v = velocity_at_end;
end

hold on;
traj_sprm_plot = plot(trajectory_sprm(:,1), trajectory_sprm(:,2), "r--", LineWidth=2);
legend([p_sprm, traj_sprm_plot], 'Straight-Line Path', 'Trajectory Path');
pause(2);

%%
fig_vis = figure;

plotGraph(vis_graph, obstacle_vertices, fig_vis, "Visibility Graph");

vis_graph = addNode(vis_graph, start_node, collisionChecker);
start_node_ID = vis_graph.n;
clf(fig_vis);
plotGraph(vis_graph, obstacle_vertices, fig_vis, "Visibility Graph");
str = "q_{start}";
text(start_node(1) + 0.02, start_node(2), str, 'FontSize', 12, 'Color', 'r');

vis_graph = addNode(vis_graph, goal_node, collisionChecker);
goal_node_ID = vis_graph.n;
clf(fig_vis);
plotGraph(vis_graph, obstacle_vertices, fig_vis, "Visibility Graph");
str1 = "q_{start}";
text(start_node(1) + 0.02, start_node(2), str1, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text(goal_node(1) + 0.02, goal_node(2), str2, 'FontSize', 12, 'Color', 'r');

[found_path_vis, path_vis, cost_vis] = ...
    MyAstar(vis_graph, start_node_ID, goal_node_ID, true);

p_vis = plotPath(fig_vis, vis_graph, path_vis, 'y');


trajectory_vis = start_node;
n = 5;
k = 4;
dt = 0.01;
T = 1;
v = 0; % initial velocity

for i = 1:size(path_vis, 2)-1
    waypoint_start = vis_graph.coord(path_vis(i));
    waypoint_end = vis_graph.coord(path_vis(i+1));
    if i < size(path_vis, 2)-1
        s = norm(waypoint_end - waypoint_start);
        a = 2*(s - v*T) / (T^2);
        velocity_at_end = v^2 + 2*a*s;
    else
        velocity_at_end = 0;
    end
    x_traj = getTrajectory(n, k, dt, T, waypoint_start, waypoint_end, ...
        [v, velocity_at_end], collisionChecker, true);
    y_traj = getTrajectory(n, k, dt, T, waypoint_start, waypoint_end, ...
        [v, velocity_at_end], collisionChecker, false);
    trajectory_vis = [trajectory_vis; x_traj, y_traj];
    v = velocity_at_end;
end

hold on;
traj_vis_plot = plot(trajectory_vis(:,1), trajectory_vis(:,2), "r--", LineWidth=2);
legend([p_vis, traj_vis_plot], 'Straight-Line Path', 'Trajectory Path');