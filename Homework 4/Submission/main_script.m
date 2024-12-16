clc; clear; close all;

obstacles = createObstacles(false, true);
obstacle_vertices = {};

for i = 1:size(obstacles, 2)
    obstacle_vertices{i, 1} = obstacles{i}.Position(:, 1);
    obstacle_vertices{i, 2} = obstacles{i}.Position(:, 2);
end

%% Test Dubins
test_dubins;

%% Compute the shortest Dubins path between two (user-defined) configurations
test_shortest_path;

%% RRT (with Dubins curves)
n_nodes = 800;
r_goal = 0.05;
r = 0.01;
seed = 7;

rrt_graph = PGraph();
fig_rrt = figure;
figure(fig_rrt);
plotGraph(rrt_graph, obstacle_vertices, r, fig_rrt, "RRT")

disp("Click a start configuration...");
disp("Click two points to indicate the position" + ...
    " and angle of the start configuration.");
x_init_1 = ginput(1);
fprintf("Start configuration is at (%.3f, %.3f). " + ...
    "Click another point to determine the angle\n", x_init_1(1), x_init_1(2));
x_init_2 = ginput(1);

theta_init = atan2(x_init_2(2)-x_init_1(2), x_init_2(1)-x_init_1(1));

if theta_init < 0
    theta_init = 2*pi + theta_init;
end

x_init = [x_init_1, theta_init];
fprintf("Start configuration is (%.3f, %.3f, %.3f deg)\n", ...
    x_init(1), x_init(2), rad2deg(x_init(3)));

disp("Click two points to indicate the position" + ...
    " and angle of the goal configuration.");
x_goal_1 = ginput(1);
fprintf("Goal configuration is at (%.3f, %.3f). " + ...
    "Click another point to determine the angle\n", x_goal_1(1), x_goal_1(2));
x_goal_2 = ginput(1);

theta_goal = atan2(x_goal_2(2)-x_goal_1(2), x_goal_2(1)-x_goal_1(1));

if theta_goal < 0
    theta_goal = 2*pi + theta_goal;
end

x_goal = [x_goal_1, theta_goal];
fprintf("Goal configuration is (%.3f, %.3f, %.3f deg)\n", ...
    x_goal(1), x_goal(2), rad2deg(x_goal(3)));

plotGoalRegion(fig_rrt, x_goal(1:2), r_goal);
str = "q_{start}";
text((x_init(1) + 0.02), (x_init(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((x_goal(1) + 0.02), (x_goal(2)), str2, 'FontSize', 12, 'Color', 'r');

tic;
[rrt_graph, found_goal_rrt, path_rrt, cost_rrt] = ...
           RRT(obstacles, x_init, x_goal, r, n_nodes, r_goal, seed, true);

dt = toc;

plotGraph(rrt_graph, obstacle_vertices, r, fig_rrt, "RRT");
p_rrt = plotPath(fig_rrt, rrt_graph, path_rrt, r, 'r');

if found_goal_rrt
    fprintf("A path of path length %.3f was found in %.3f seconds\n", cost_rrt, dt);
end

pause(2);
