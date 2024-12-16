function [path, path_length] = shortestDubins(fig, x_init, x_goal, r, collisionChecker, plot_circles, plot_tangents, plot_path)
    % Function to compute the shortest dubins path between two
    % configurations that is collision free
    
    path = [];
    path_length = Inf;
    f = [];

    if plot_path
        figure(fig);
        axis square;
    end

    %fprintf("ro function = %f\n", r);

    dist = norm(x_init(1:2) - x_goal(1:2));
    e = 0.001;

    if dist + e < 4.1*r && ...
            collisionFree(collisionChecker, x_init, x_goal, r, "S", 0.01, 0.01)
        path = "S";
        path_length = dist;
        %fprintf("r = %f, dist = %f\n", r, dist);
        return;
    elseif dist + e < 4.1*r
        %fprintf("r = %f, dist = %f\n", r, dist);
        return;
    end

    [path2, path_length2] = LSL(fig, x_init, x_goal, r, false, false, false);

    if path_length2 < path_length && ...
            collisionFree(collisionChecker, x_init, x_goal, r, path2, 0.01, 0.01)
        path = path2;
        path_length = path_length2;
        f = @LSL;
    end

    [path2, path_length2] = RSR(fig, x_init, x_goal, r, false, false, false);

    if path_length2 < path_length && ...
            collisionFree(collisionChecker, x_init, x_goal, r, path2, 0.01, 0.01)
        path = path2;
        path_length = path_length2;
        f = @RSR;
    end

    [path2, path_length2] = LSR(fig, x_init, x_goal, r, false, false, false);

    if path_length2 < path_length && ...
            collisionFree(collisionChecker, x_init, x_goal, r, path2, 0.01, 0.01)
        path = path2;
        path_length = path_length2;
        f = @LSR;
    end

    [path2, path_length2] = RSL(fig, x_init, x_goal, r, false, false, false);

    if path_length2 < path_length && ...
            collisionFree(collisionChecker, x_init, x_goal, r, path2, 0.01, 0.01)
        path = path2;
        path_length = path_length2;
        f = @RSL;
    end

    if ~isempty(f)
        f(fig, x_init, x_goal, r, plot_circles, plot_tangents, plot_path);
    end


end