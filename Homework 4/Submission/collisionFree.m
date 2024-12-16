function cf = collisionFree(collisionChecker, u, v, r, dubinsCurve, step, theta_step)
    % function to check if the dubins curve between nodes u and v
    % of combination dubinsCurve is collision free

    if isempty(collisionChecker)
        cf = true;
        return;
    end

    % centers of 4 circles
    if dubinsCurve == "LSL" || dubinsCurve == "RSR" || dubinsCurve == "LSR" ...
        || dubinsCurve == "RSL"
        c_init_left = [u(1) - r*sin(u(3)), u(2) + r*cos(u(3))];
        c_init_right = [u(1) + r*sin(u(3)), u(2) - r*cos(u(3))];
    
        c_goal_left = [v(1) - r*sin(v(3)), v(2) + r*cos(v(3))];
        c_goal_right = [v(1) + r*sin(v(3)), v(2) - r*cos(v(3))];
    end


    if dubinsCurve == "LSL"
        tangentPoints = externalTangents(c_init_left, c_goal_left, r, true, false, 'm');
    
        p1_init = u(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = atan2(p2_init(2) - c_init_left(2), p2_init(1) - c_init_left(1)) - ...
            atan2(p1_init(2) - c_init_left(2), p1_init(1) - c_init_left(1));
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
           
        start_th = 3*pi/2+u(3);
        th = start_th:theta_step:start_th+phi_init;

        for i = 1:length(th)
            q = [c_init_left(1) + r*cos(th(i)) c_init_left(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        p1_goal = v(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = atan2(p1_goal(2) - c_goal_left(2), p1_goal(1) - c_goal_left(1)) -...
            atan2(p2_goal(2) - c_goal_left(2), p2_goal(1) - c_goal_left(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
        start_th = 3*pi/2+v(3)-phi_goal;
        th = start_th:theta_step:start_th+phi_goal;

        for i = 1:length(th)
            q = [c_goal_left(1) + r*cos(th(i)) c_goal_left(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        u = tangentPoints(1,:);
        v = tangentPoints(2,:);

        vec = (v - u);
        normvec = norm(v - u);
        uvec = vec/normvec;
    
        distance = step;
        cf = collisionChecker.isFree(u+uvec*distance);
        while cf && (distance < normvec)
            distance = distance+step;
            cf = collisionChecker.isFree(u+uvec*distance);
        end

        
    elseif dubinsCurve == "RSR"
        tangentPoints = externalTangents(c_init_right, c_goal_right, r, false, false, 'm');
        
        p1_init = u(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = -atan2(p2_init(2) - c_init_right(2), p2_init(1) - c_init_right(1)) + ...
            atan2(p1_init(2) - c_init_right(2), p1_init(1) - c_init_right(1));
    
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
        
        
        goal_th = pi/2+u(3);
        th = goal_th-phi_init:theta_step:goal_th;

        for i = 1:length(th)
            q = [c_init_right(1) + r*cos(th(i)) c_init_right(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        p1_goal = v(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = - atan2(p1_goal(2) - c_goal_right(2), p1_goal(1) - c_goal_right(1)) +...
            atan2(p2_goal(2) - c_goal_right(2), p2_goal(1) - c_goal_right(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
    
        goal_th = pi/2+v(3)+phi_goal;
        th = goal_th-phi_goal:theta_step:goal_th;

        for i = 1:length(th)
            q = [c_goal_right(1) + r*cos(th(i)) c_goal_right(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        u = tangentPoints(1,:);
        v = tangentPoints(2,:);

        vec = (v - u);
        normvec = norm(v - u);
        uvec = vec/normvec;
    
        distance = step;
        cf = collisionChecker.isFree(u+uvec*distance);
        while cf && (distance < normvec)
            distance = distance+step;
            cf = collisionChecker.isFree(u+uvec*distance);
        end


    elseif dubinsCurve == "LSR"
        tangentPoints = internalTangents(c_init_left, c_goal_right, r, false, false, 'b');
 
        p1_init = u(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = atan2(p2_init(2) - c_init_left(2), p2_init(1) - c_init_left(1)) - ...
            atan2(p1_init(2) - c_init_left(2), p1_init(1) - c_init_left(1));
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
        
        
        start_th = 3*pi/2+u(3);
        th = start_th:theta_step:start_th+phi_init;

        for i = 1:length(th)
            q = [c_init_left(1) + r*cos(th(i)) c_init_left(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        p1_goal = v(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = - atan2(p1_goal(2) - c_goal_right(2), p1_goal(1) - c_goal_right(1)) +...
            atan2(p2_goal(2) - c_goal_right(2), p2_goal(1) - c_goal_right(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
        goal_th = pi/2+v(3)+phi_goal;
        th = goal_th-phi_goal:theta_step:goal_th;

        for i = 1:length(th)
            q = [c_goal_right(1) + r*cos(th(i)) c_goal_right(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        u = tangentPoints(1,:);
        v = tangentPoints(2,:);

        vec = (v - u);
        normvec = norm(v - u);
        uvec = vec/normvec;
    
        distance = step;
        cf = collisionChecker.isFree(u+uvec*distance);
        while cf && (distance < normvec)
            distance = distance+step;
            cf = collisionChecker.isFree(u+uvec*distance);
        end


    elseif dubinsCurve == "RSL"
        tangentPoints = internalTangents(c_init_right, c_goal_left, r, true, false, 'b');
    
        p1_init = u(1:2);
        p2_init = tangentPoints(1,:);
    
        phi_init = -atan2(p2_init(2) - c_init_right(2), p2_init(1) - c_init_right(1)) + ...
            atan2(p1_init(2) - c_init_right(2), p1_init(1) - c_init_right(1));
    
    
        if phi_init < 0
            phi_init = phi_init + 2*pi;
        end
        
        
        goal_th = pi/2+u(3);
        th = goal_th-phi_init:theta_step:goal_th;

        for i = 1:length(th)
            q = [c_init_right(1) + r*cos(th(i)) c_init_right(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        p1_goal = v(1:2);
        p2_goal = tangentPoints(2,:);
    
        phi_goal = atan2(p1_goal(2) - c_goal_left(2), p1_goal(1) - c_goal_left(1)) -...
            atan2(p2_goal(2) - c_goal_left(2), p2_goal(1) - c_goal_left(1));
    
        if phi_goal < 0
            phi_goal = phi_goal + 2*pi;
        end
    
        start_th = 3*pi/2+v(3)-phi_goal;
        th = start_th:theta_step:start_th+phi_goal;

        for i = 1:length(th)
            q = [c_goal_left(1) + r*cos(th(i)) c_goal_left(2)+r*sin(th(i))];
            cf = collisionChecker.isFree(q);

            if ~cf
                break;
            end
        end

        if ~cf
            return;
        end

        u = tangentPoints(1,:);
        v = tangentPoints(2,:);

        vec = (v - u);
        normvec = norm(v - u);
        uvec = vec/normvec;
    
        distance = step;
        cf = collisionChecker.isFree(u+uvec*distance);
        while cf && (distance < normvec)
            distance = distance+step;
            cf = collisionChecker.isFree(u+uvec*distance);
        end

    else
        v = v(1:2);
        u = u(1:2);
        
        vec = (v - u);
        normvec = norm(v - u);
        uvec = vec/normvec;
    
        distance = step;
        cf = collisionChecker.isFree(u+uvec*distance);
        while cf && (distance < normvec)
            distance = distance+step;
            cf = collisionChecker.isFree(u+uvec*distance);
        end
    end

    
end