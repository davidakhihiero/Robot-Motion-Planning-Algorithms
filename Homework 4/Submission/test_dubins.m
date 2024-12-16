wait_time = 2;
%% Test Dubins LSL
fig = figure;
hold on;
axis square;
title("Testing Dubins LSL");
disp("Testing Dubins LSL");
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

r = 0.05;

[path, path_length] = LSL(fig, x_init, x_goal, r, true, true, true);
pause(wait_time);
hold off;

%% Test Dubins RSR
fig = figure;
hold on;
axis square;
title("Testing Dubins RSR");
disp("Testing Dubins RSR");
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

r = 0.05;

[path, path_length] = RSR(fig, x_init, x_goal, r, true, true, true);
pause(wait_time);
hold off;

%% Test Dubins LSR
fig = figure;
hold on;
axis square;
title("Testing Dubins LSR");
disp("Testing Dubins LSR");
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

r = 0.05;

[path, path_length] = LSR(fig, x_init, x_goal, r, true, true, true);
pause(wait_time);
hold off;

%% Test Dubins RSL
fig = figure;
hold on;
axis square;
title("Testing Dubins RSL");
disp("Testing Dubins RSL");
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

r = 0.05;

[path, path_length] = RSL(fig, x_init, x_goal, r, true, true, true);
pause(wait_time);
hold off;