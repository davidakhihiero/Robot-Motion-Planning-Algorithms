function [path_length, plot_data] = dubinsPath(fig, x_init, x_goal, r, combination, style, linewidth, plot_tangents, plot_path)
    % function to plot dubins curves for LSL, RSR, LSR, RSL
    % x_init: initial configuration (x1, y1, theta1)
    % x_goal: final configuration (x2, y2, theta2)
    % r: minimum turning radius
    % combination: type of dubins curve

    if plot_path
        figure(fig);
        axis([0 1 0 1]);
        axis square;
    end

    plot_data = [];
    
    path_length = 0;
    theta_step = pi/100;

    % centers of 4 circles
    c_init_left = [x_init(1) - r*sin(x_init(3)), x_init(2) + r*cos(x_init(3))];
    c_init_right = [x_init(1) + r*sin(x_init(3)), x_init(2) - r*cos(x_init(3))];

    c_goal_left = [x_goal(1) - r*sin(x_goal(3)), x_goal(2) + r*cos(x_goal(3))];
    c_goal_right = [x_goal(1) + r*sin(x_goal(3)), x_goal(2) - r*cos(x_goal(3))];



    if combination == "LSL"
        tangentPoints = externalTangents(c_init_left, c_goal_left, r, true, plot_tangents, 'm');
    
        p1_init = x_init(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = atan2(p2_init(2) - c_init_left(2), p2_init(1) - c_init_left(1)) - ...
            atan2(p1_init(2) - c_init_left(2), p1_init(1) - c_init_left(1));
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
           
        start_th = 3*pi/2+x_init(3);
        th = start_th:theta_step:start_th+phi_init;
    
        path_length = path_length + r*phi_init;
    
        if plot_path
            circle(c_init_left(1), c_init_left(2), r, th, style, linewidth);
        end
    
        p1_goal = x_goal(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = atan2(p1_goal(2) - c_goal_left(2), p1_goal(1) - c_goal_left(1)) -...
            atan2(p2_goal(2) - c_goal_left(2), p2_goal(1) - c_goal_left(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
        start_th = 3*pi/2+x_goal(3)-phi_goal;
        th = start_th:theta_step:start_th+phi_goal;
    
        path_length = path_length + r*phi_goal;
    
        if plot_path
            circle(c_goal_left(1), c_goal_left(2), r, th, style, linewidth);
            plot([p2_init(1), p2_goal(1)], [p2_init(2), p2_goal(2)], style, LineWidth=linewidth);
        end
            
        path_length = path_length + norm(p2_goal - p2_init);

    elseif combination == "RSR"
        tangentPoints = externalTangents(c_init_right, c_goal_right, r, false, plot_tangents, 'm');
        
        p1_init = x_init(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = -atan2(p2_init(2) - c_init_right(2), p2_init(1) - c_init_right(1)) + ...
            atan2(p1_init(2) - c_init_right(2), p1_init(1) - c_init_right(1));
    
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
        
        
        goal_th = pi/2+x_init(3);
        th = goal_th-phi_init:theta_step:goal_th;
    
        path_length = path_length + r*phi_init;
    
        if plot_path
            circle(c_init_right(1), c_init_right(2), r, th, style, linewidth);
        end
    
        p1_goal = x_goal(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = - atan2(p1_goal(2) - c_goal_right(2), p1_goal(1) - c_goal_right(1)) +...
            atan2(p2_goal(2) - c_goal_right(2), p2_goal(1) - c_goal_right(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
    
        goal_th = pi/2+x_goal(3)+phi_goal;
        th = goal_th-phi_goal:theta_step:goal_th;
    
        path_length = path_length + r*phi_goal;
    
        if plot_path
            circle(c_goal_right(1), c_goal_right(2), r, th, style, linewidth);
            plot([p2_init(1), p2_goal(1)], [p2_init(2), p2_goal(2)], style, LineWidth=linewidth);
        end

        path_length = path_length + norm(p2_goal - p2_init);

    elseif combination == "LSR"
        tangentPoints = internalTangents(c_init_left, c_goal_right, r, false, plot_tangents, 'b');
 
        p1_init = x_init(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = atan2(p2_init(2) - c_init_left(2), p2_init(1) - c_init_left(1)) - ...
            atan2(p1_init(2) - c_init_left(2), p1_init(1) - c_init_left(1));
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
        
        
        start_th = 3*pi/2+x_init(3);
        th = start_th:theta_step:start_th+phi_init;
    
        path_length = path_length + r*phi_init;
    
        if plot_path
            circle(c_init_left(1), c_init_left(2), r, th, style, linewidth);
        end
    
        p1_goal = x_goal(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = - atan2(p1_goal(2) - c_goal_right(2), p1_goal(1) - c_goal_right(1)) +...
            atan2(p2_goal(2) - c_goal_right(2), p2_goal(1) - c_goal_right(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
        goal_th = pi/2+x_goal(3)+phi_goal;
        th = goal_th-phi_goal:theta_step:goal_th;
    
        path_length = path_length + r*phi_goal;
    
        if plot_path
            circle(c_goal_right(1), c_goal_right(2), r, th, style, linewidth);
            plot([p2_init(1), p2_goal(1)], [p2_init(2), p2_goal(2)], style, LineWidth=linewidth);
        end
  
        path_length = path_length + norm(p2_goal - p2_init); 

    elseif combination == "RSL"
        tangentPoints = internalTangents(c_init_right, c_goal_left, r, true, plot_tangents, 'b');
    
        p1_init = x_init(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = -atan2(p2_init(2) - c_init_right(2), p2_init(1) - c_init_right(1)) + ...
            atan2(p1_init(2) - c_init_right(2), p1_init(1) - c_init_right(1));
    
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
        
        
        goal_th = pi/2+x_init(3);
        th = goal_th-phi_init:theta_step:goal_th;
    
        path_length = path_length + r*phi_init;
    
        if plot_path
            plot_data(1) = circle(c_init_right(1), c_init_right(2), r, th, style, linewidth);
        end
    
        p1_goal = x_goal(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = atan2(p1_goal(2) - c_goal_left(2), p1_goal(1) - c_goal_left(1)) -...
            atan2(p2_goal(2) - c_goal_left(2), p2_goal(1) - c_goal_left(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
        start_th = 3*pi/2+x_goal(3)-phi_goal;
        th = start_th:theta_step:start_th+phi_goal;
    
        path_length = path_length + r*phi_goal;
    
        if plot_path
            plot_data(2) = circle(c_goal_left(1), c_goal_left(2), r, th, style, linewidth);
            plot_data(3) = plot([p2_init(1), p2_goal(1)], [p2_init(2), p2_goal(2)], style, LineWidth=linewidth);
        end
    
        path_length = path_length + norm(p2_goal - p2_init);
    else
        if plot_path
            plot([x_init(1), x_goal(1)], [x_init(2), x_goal(2)], style, LineWidth=linewidth);
        end

    end

end