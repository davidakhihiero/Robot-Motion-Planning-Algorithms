clc; clear; close all;

obstacles = createObstacles(false, true);

obstacle_vertices = {};

for i = 1:size(obstacles, 2)
    obstacle_vertices{i, 1} = obstacles{i}.Position(:, 1);
    obstacle_vertices{i, 2} = obstacles{i}.Position(:, 2);
end

%% RRT*
n_nodes = 800;
r_goal = 0.05;
r = 0.1;
seed = 7;

rrt_star_graph = PGraph();
fig_rrt_star = figure;
figure(fig_rrt_star);
plotGraph(rrt_star_graph, obstacle_vertices, fig_rrt_star, "RRT*")

disp("Click a start configuration...");
x_init = ginput(1);
x_init = x_init';
fprintf("Start configuration [%f, %f] clicked\n", x_init(1), x_init(2));
disp("Click a goal configuration...");
x_goal = ginput(1);
x_goal = x_goal';
fprintf("Goal configuration [%f, %f] clicked\n", x_goal(1), x_goal(2));

plotGoalRegion(fig_rrt_star, x_goal, r_goal);
str = "q_{start}";
text((x_init(1) + 0.02), (x_init(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((x_goal(1) + 0.02), (x_goal(2)), str2, 'FontSize', 12, 'Color', 'r');


[rrt_star_graph, found_goal_rrt_star, path_rrt_star, cost_rrt_star] = ...
           RRTstar(obstacles, x_init, x_goal, n_nodes, r, r_goal, Inf, seed, true);

plotGraph(rrt_star_graph, obstacle_vertices, fig_rrt_star, "RRT*");
p_rrt_star = plotPath(fig_rrt_star, rrt_star_graph, path_rrt_star, 'k');
pause(2);

%% Informed RRT*
N = 800;
r_goal = 0.05;
r = 0.1;
seed = 7;

informed_rrt_star_graph = PGraph();
fig_informed_rrt_star = figure;
figure(fig_informed_rrt_star);
plotGraph(informed_rrt_star_graph, obstacle_vertices, fig_informed_rrt_star, "Informed RRT*")

disp("Click a start configuration...");
x_init = ginput(1);
x_init = x_init';
fprintf("Start configuration [%f, %f] clicked\n", x_init(1), x_init(2));
disp("Click a goal configuration...");
x_goal = ginput(1);
x_goal = x_goal';
fprintf("Goal configuration [%f, %f] clicked\n", x_goal(1), x_goal(2));

plotGoalRegion(fig_informed_rrt_star, x_goal, r_goal);
str = "q_{start}";
text((x_init(1) + 0.02), (x_init(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((x_goal(1) + 0.02), (x_goal(2)), str2, 'FontSize', 12, 'Color', 'r');


[informed_rrt_star_graph, found_goal_informed_rrt_star, path_informed_rrt_star, cost_informed_rrt_star] = ...
           InformedRRTstar(obstacles, x_init, x_goal, N, r, r_goal, Inf, seed, true);

plotGraph(informed_rrt_star_graph, obstacle_vertices, fig_informed_rrt_star, "Informed RRT*");
p_informed_rrt_star = plotPath(fig_informed_rrt_star, informed_rrt_star_graph, path_informed_rrt_star, 'k');
pause(2);

%% Comparisons
compare = true;
time_to_improve_path = 2;

if compare
    fig = figure;
    obstacles = createObstacles(false, true);
    obstacle_vertices = {};
    
    for i = 1:size(obstacles, 2)
        obstacle_vertices{i, 1} = obstacles{i}.Position(:, 1);
        obstacle_vertices{i, 2} = obstacles{i}.Position(:, 2);
    end
    start = [0.05; 0.03]; %[0.1; 0.1];
    goal = [0.9;  0.9];
    


    % sPRM
    n_nodes = 800;
    r = 0.2;
    seed = 7;

    tic;
    sprm_graph = sPRMLearning(obstacles, n_nodes, r, seed, true);
    dt_sprm_preprocessing = toc;

    
    k = 5;
    
    tic;
    [sprm_graph, found_goal_sprm, path_sprm, cost_sprm] = ...
                                      queryRoadmap(sprm_graph, obstacles, start, goal, "sPRM", k, true);
    dt_sprm = toc;

    % RRT
    n_nodes = 800;
    r_goal = 0.05;
    seed = 7;


    tic;
    [rrt_graph, found_goal_rrt, path_rrt, cost_rrt] = ...
               RRT(obstacles, start, goal, n_nodes, r_goal, time_to_improve_path, seed, true);
    dt_rrt = toc;

    % RRT*
    n_nodes = 800;
    r = 0.1;
    r_goal = 0.05;
    seed = 7;


    tic;
    [rrt_star_graph, found_goal_rrt_star, path_rrt_star, cost_rrt_star] = ...
               RRTstar(obstacles, start, goal, n_nodes, r, r_goal, time_to_improve_path, seed, true);
    dt_rrt_star = toc;

    % Informed RRT*
    N = 800;
    r = 0.1;
    r_goal = 0.05;
    seed = 7;


    tic;
    [informed_rrt_star_graph, found_goal_informed_rrt_star, path_informed_rrt_star, ...
        cost_informed_rrt_star] = ...
               InformedRRTstar(obstacles, start, goal, N, r, r_goal, time_to_improve_path, seed, true);
    dt_informed_rrt_star = toc;
    
    
    if found_goal_sprm
        fprintf("An sPRM graph was preprocessed in %f seconds " + ...
            "and a path of length %f was found in %f seconds\n", ...
            dt_sprm_preprocessing, cost_sprm, dt_sprm);
    else
        disp("No sPRM path was found");
    end

    if found_goal_rrt
        fprintf("An RRT path of length %f was found in %f seconds to a goal " + ...
            "region of radius %.3f\n", ...
            cost_rrt, dt_rrt, r_goal);
    else
        disp("No RRT path was found");
    end

    if found_goal_rrt_star
        fprintf("An RRT* path of length %f was found in %f seconds to a goal " + ...
            "region of radius %.3f\n", ...
            cost_rrt_star, dt_rrt_star, r_goal);
    else
        disp("No RRT* path was found");
    end

    if found_goal_informed_rrt_star
        fprintf("An informed RRT* path of length %f was found in %f seconds to a goal " + ...
            "region of radius %.3f\n", ...
            cost_informed_rrt_star, dt_informed_rrt_star, r_goal);
    else
        disp("No informed RRT* path was found");
    end

    hold on;
    plotObstacles(fig, obstacle_vertices);
    str = "q_{start}";
    text((start(1) + 0.02), (start(2)), str, 'FontSize', 12, 'Color', 'r');
    str2 = "q_{goal}";
    text((goal(1) + 0.02), (goal(2)), str2, 'FontSize', 12, 'Color', 'r');
  
    p_sprm = plotPath(fig, sprm_graph, path_sprm, 'y');
    % plotGraph(sprm_graph, obstacle_vertices, fig, "Graph");
    p_rrt = plotPath(fig, rrt_graph, path_rrt, 'r');

    p_rrt_star = plotPath(fig, rrt_star_graph, path_rrt_star, 'g');

    p_informed_rrt_star = plotPath(fig, informed_rrt_star_graph, path_informed_rrt_star, 'k--');

    plotGoalRegion(fig, goal, r_goal);
    % plotGraph(rrt_star_graph, obstacle_vertices, fig, "Graph");
    legend([p_sprm, p_rrt, p_rrt_star, p_informed_rrt_star], 'sPRM', 'RRT', 'RRT*', 'Informed RRT*');
    grid;
end
