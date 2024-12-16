function q = informedSampleFree(collisionChecker, x_start, x_goal, c_best, env_size)
    % Gammell, J. D., Srinivasa, S. S., & Barfoot, T. D. (2014, September). Informed RRT*: Optimal sampling-
    % based path planning focused via direct sampling of an admissible ellipsoidal heuristic. In 2014 IEEE/RSJ
    % International Conference on Intelligent Robots and Systems (pp. 2997-3004). IEEE.'
    if c_best == Inf
        q = rand(1, 2) * env_size;
    
        while ~collisionChecker.isFree(q)
            q = rand(1, 2) * env_size;
        end
    else
        c_min = norm(x_goal - x_start);
        x_center = (x_goal + x_start) / 2;
        theta = atan2(x_goal(2) - x_start(2), x_goal(1) - x_start(1));
        C = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        L = [c_best / 2 0; 0 sqrt(c_best^2 - c_min^2) / 2];
        r = sqrt(rand());
        angle=rand()*2*pi;
        q = [r*cos(angle); r*sin(angle)];
        q = C*L*q + x_center;

        q = q';
    end

end