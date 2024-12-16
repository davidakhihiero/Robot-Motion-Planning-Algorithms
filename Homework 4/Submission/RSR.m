function [path, path_length] = RSR(fig, x_init, x_goal, r, plot_circles, plot_tangents, plot_path)
    % Function to compute Dubins RSR
    % Giese, A. (2012, October). A comprehensive, step-by-step tutorial on computing dubin’s curves.
    % x_init: initial configuration (x1, y1, theta1)
    % x_goal: final configuration (x2, y2, theta2)
    % r: minimum turning radius
    path = "RSR";
    path_length = 0;
    theta_step = pi/100;
    scale = 2*r; 

    % centers of 4 circles
    c_init_left = [x_init(1) - r*sin(x_init(3)), x_init(2) + r*cos(x_init(3))];
    c_init_right = [x_init(1) + r*sin(x_init(3)), x_init(2) - r*cos(x_init(3))];

    c_goal_left = [x_goal(1) - r*sin(x_goal(3)), x_goal(2) + r*cos(x_goal(3))];
    c_goal_right = [x_goal(1) + r*sin(x_goal(3)), x_goal(2) - r*cos(x_goal(3))];

    % plot circles
    if plot_circles
        plot(x_init(1), x_init(2), 'r*');
        hold on
        quiver(x_init(1), x_init(2), scale * cos(x_init(3)), scale * sin(x_init(3)));
        plot(x_goal(1), x_goal(2), 'r*');
        quiver(x_goal(1), x_goal(2), scale * cos(x_goal(3)), scale * sin(x_goal(3)));
        %axis equal

        circle(c_init_left(1), c_init_left(2), r, 0:theta_step:2*pi, 'r', 1);
        circle(c_init_right(1), c_init_right(2), r, 0:theta_step:2*pi, 'r', 1);
        circle(c_goal_left(1), c_goal_left(2), r, 0:theta_step:2*pi, 'g', 1);
        circle(c_goal_right(1), c_goal_right(2), r, 0:theta_step:2*pi, 'g', 1);
    end

    path_length = dubinsPath(fig, x_init, x_goal, r, "RSR", "k-", 2, plot_tangents, plot_path);
end