clc; clear; close all;

obstacles = createObstacles(false, true);

%% Discrete Navigation Function (Wavefront)
resolution = 0.002;
env_size = 1;
bitmap = createGridBasedEnvironment(obstacles, 1, resolution);

fig_wavefront = figure;

imagesc(1 - (bitmap));
title("Wavefront");
axis([0 env_size/resolution 0 env_size/resolution]); 

% axis image; 

xticks(0:0.1*env_size/resolution:env_size/resolution); 
yticks(0:0.1*env_size/resolution:env_size/resolution); 

xticklabels(0:0.1:env_size);
yticklabels(env_size:-0.1:0);
hold on


goal = [0.9;  0.9];
start = ginput(1) * resolution;
start = start';
start(2) = env_size - start(2);
% start = [0.1;0.1]
% start = [0.05; 0.03];

[path_wavefront, navigation_function, cost_wavefront] = wavefront(bitmap, start, goal, resolution, true);

figure(fig_wavefront);

imagesc(navigation_function)

plot(path_wavefront(:, 1), path_wavefront(:, 2), 'r-', 'LineWidth', 2);
str = "q_{start}";
text((start(1) + 0.02) * size(navigation_function,1), (env_size - start(2)) * size(navigation_function, 1), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((goal(1) + 0.02) * size(navigation_function,1), (env_size - goal(2)) * size(navigation_function,1), str2, 'FontSize', 12, 'Color', 'r');
pause(2);



%% PRM
n_nodes = 200;
r = 0.2;
seed = 7;
prm_graph = PRMLearning(obstacles, n_nodes, r, seed, true);

obstacle_vertices = {};

for i = 1:size(obstacles, 2)
    obstacle_vertices{i, 1} = obstacles{i}.Position(:, 1);
    obstacle_vertices{i, 2} = obstacles{i}.Position(:, 2);
end

fig_prm = figure;
figure(fig_prm);
plotGraph(prm_graph, obstacle_vertices, fig_prm, "PRM")

k = 5;
disp("Click a start configuration...");
start = ginput(1);
start = start';
fprintf("Start configuration [%f, %f] clicked\n", start(1), start(2));
disp("Click a goal configuration...");
goal = ginput(1);
goal = goal';
fprintf("Goal configuration [%f, %f] clicked\n", goal(1), goal(2));
[prm_graph, found_goal_prm, path_prm, cost_prm] = ...
                                  queryRoadmap(prm_graph, obstacles, start, goal, "PRM", k, true);
plotGraph(prm_graph, obstacle_vertices, fig_prm, "PRM");
str = "q_{start}";
text((start(1) + 0.02), (start(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((goal(1) + 0.02), (goal(2)), str2, 'FontSize', 12, 'Color', 'r');
hold on;

p_prm = plotPath(fig_prm, prm_graph, path_prm, 'g');
pause(2);

%% sPRM 
n_nodes = 200;
r = 0.2;
seed = 7;
sprm_graph = sPRMLearning(obstacles, n_nodes, r, seed, true);

fig_sprm = figure;
figure(fig_sprm);
plotGraph(sprm_graph, obstacle_vertices, fig_sprm, "sPRM");

k = 5;
disp("Click a start configuration...");
start = ginput(1);
start = start';
fprintf("Start configuration [%f, %f] clicked\n", start(1), start(2));
disp("Click a goal configuration...");
goal = ginput(1);
goal = goal';
fprintf("Goal configuration [%f, %f] clicked\n", goal(1), goal(2));
[sprm_graph, found_goal_sprm, path_sprm, cost_sprm] = ...
                                  queryRoadmap(sprm_graph, obstacles, start, goal, "sPRM", k, true);
plotGraph(sprm_graph, obstacle_vertices, fig_sprm, "sPRM");
str = "q_{start}";
text((start(1) + 0.02), (start(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((goal(1) + 0.02), (goal(2)), str2, 'FontSize', 12, 'Color', 'r');
hold on;

p_sprm = plotPath(fig_sprm, sprm_graph, path_sprm, 'y');
pause(2);

%% RRT
n_nodes = 600;
r_goal = 0.05;
seed = 7;

rrt_graph = PGraph();
fig_rrt = figure;
figure(fig_rrt);
plotGraph(rrt_graph, obstacle_vertices, fig_rrt, "RRT")

disp("Click a start configuration...");
x_init = ginput(1);
x_init = x_init';
fprintf("Start configuration [%f, %f] clicked\n", x_init(1), x_init(2));
disp("Click a goal configuration...");
x_goal = ginput(1);
x_goal = x_goal';
fprintf("Goal configuration [%f, %f] clicked\n", x_goal(1), x_goal(2));

plotGoalRegion(fig_rrt, x_goal, r_goal);
str = "q_{start}";
text((x_init(1) + 0.02), (x_init(2)), str, 'FontSize', 12, 'Color', 'r');
str2 = "q_{goal}";
text((x_goal(1) + 0.02), (x_goal(2)), str2, 'FontSize', 12, 'Color', 'r');


[rrt_graph, found_goal_rrt, path_rrt, cost_rrt] = ...
           RRT(obstacles, x_init, x_goal, n_nodes, r_goal, seed, true);

plotGraph(rrt_graph, obstacle_vertices, fig_rrt, "RRT");
p_rrt = plotPath(fig_rrt, rrt_graph, path_rrt, 'k');
pause(2);

%% Comparisons
compare = true;

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
    
    % Wavefront
    resolution = 0.002;
    env_size = 1;
    tic;
    bitmap = createGridBasedEnvironment(obstacles, 1, resolution);
    dt_wavefront_preprocessing = toc;

    tic;
    [path_wavefront, navigation_function, cost_wavefront] = wavefront(bitmap, start, goal, resolution, true);
    dt_wavefront = toc;

    % PRM
    n_nodes = 600;
    r = 0.2;
    seed = 7;

    tic;
    prm_graph = PRMLearning(obstacles, n_nodes, r, seed, true);
    dt_prm_preprocessing = toc;

    k = 5;
    tic;
    [prm_graph, found_goal_prm, path_prm, cost_prm] = ...
                                      queryRoadmap(prm_graph, obstacles, start, goal, "PRM", k, true);
    dt_prm = toc;

    % sPRM
    n_nodes = 600;
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
    n_nodes = 600;
    r_goal = 0.05;
    seed = 7;


    tic;
    [rrt_graph, found_goal_rrt, path_rrt, cost_rrt] = ...
               RRT(obstacles, start, goal, n_nodes, r_goal, seed, true);
    dt_rrt = toc;
    
        
    if size(path_wavefront, 1) > 0
        fprintf("A wavefront navigation function was preprocessed in %f seconds " + ...
            "and a path of length %f was found in %f seconds\n", ...
            dt_wavefront_preprocessing, cost_wavefront, dt_wavefront);
    else
        disp("No wavefront path was found");
    end

    if found_goal_prm
        fprintf("A PRM graph was preprocessed in %f seconds " + ...
            "and a path of length %f was found in %f seconds\n", ...
            dt_prm_preprocessing, cost_prm, dt_prm);
    else
        disp("No PRM path was found");
    end

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

    hold on;
    plotObstacles(fig, obstacle_vertices);
    str = "q_{start}";
    text((start(1) + 0.02), (start(2)), str, 'FontSize', 12, 'Color', 'r');
    str2 = "q_{goal}";
    text((goal(1) + 0.02), (goal(2)), str2, 'FontSize', 12, 'Color', 'r');
    p_wavefront = plotWavefrontPath(fig, resolution, env_size, path_wavefront, 'r');
    p_prm = plotPath(fig, prm_graph, path_prm, 'g');
    % plotGraph(prm_graph, obstacle_vertices, fig, "Graph");
    p_sprm = plotPath(fig, sprm_graph, path_sprm, 'y');
    % plotGraph(sprm_graph, obstacle_vertices, fig, "Graph");
    p_rrt = plotPath(fig, rrt_graph, path_rrt, 'k');
    plotGoalRegion(fig, goal, r_goal);
    % plotGraph(rrt_graph, obstacle_vertices, fig, "Graph");
    legend([p_wavefront, p_prm, p_sprm, p_rrt], 'Wavefront', 'PRM', 'sPRM', 'RRT');
    grid;
end