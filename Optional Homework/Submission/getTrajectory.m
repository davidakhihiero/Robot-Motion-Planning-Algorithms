function traj = getTrajectory(n, k, dt, T, x1, x2, v, collisionChecker, x_coord)
    % n : order of the polynomial
    % k: derivative being optimized
    % dt: time step
    % T: final time
    % x1: initial position constraint
    % x2: final position constraint
    % v: velocity constraints

    % forming the Hessian
    for i = 0 : n-1
        for j = 0 : n-1
           
            if i>=k && j>=k
                product = 1;
                for m = 0 : k-1
                    product = product * (i-m) * (j-m);
                end
    
                H(i+1,j+1) = 2*product*(T^(i+j-2*k+1))/(i+j-2*k+1);
            else
                H(i+1,j+1) =  0;
            end
        end
    end

    % Equality Constraints
    % Position constraints

    kp = 0; % Derivative of the constraint
    t = 0; % Time of this constraint
    
    for j = 0 : n-1 
        if j >= kp
            product = 1;
            for m = 0 : kp-1
                product = product * (j-m);
            end
            A(j+1) = product*t^(j-kp);
        else
            A(j+1) = 0;
        end
    end
    
    if x_coord
        b = x1(1); % Initial position
    else
        b = x1(2); % Initial position
    end
    
    Ae = A;
    be = b;
    
    kp = 0; % Derivative of the constraint
    t = T; % Time of this constraint
    
    for j = 0 : n-1 
        if j >= kp
            product = 1;
            for m = 0 : kp-1
                product = product * (j-m);
            end
            A(j+1) = product*t^(j-kp);
        else
            A(j+1) = 0;
        end
    end
    
    if x_coord
        b = x2(1); % Final position
    else
        b = x2(2); % Final position
    end
    
    Ae = [Ae; A];
    be = [be; b];

    % Velocity constraints

    k = 1; % Derivative of the constraint
    t = 0; % Time of this constraint
    
    for j = 0 : n-1 
        if j >= k
            product = 1;
            for m = 0 : k-1
                product = product * (j-m);
            end
            A(j+1)= product*t^(j-k);
        else
            A(j+1)=0;
        end
    end
    
    b = v(1); % Initial velocity
    
    Ae = [Ae; A];
    be = [be; b];
    
    kv = 1; % Derivative of the constraint
    t = T; % Time of this constraint
    
    for j = 0 : n-1 
        if j >= kv
            product = 1;
            for m = 0 : kv-1
                product = product * (j-m);
            end
            A(j+1) = product*t^(j - kv);
        else
            A(j+1) = 0;
        end
    end
    
    b = v(2); % Final velocity
    
    Ae = [Ae; A];
    be = [be; b];


    % Inequality Constraints
    Ai=[];
    bi=[];

    ki=0; % Derivative of the constraint
    
    vec = x2 - x1;
    step = 0.01;

    p = x1; % move point along straight path to check obstacles to the sides of the path

    for t = 0 : dt : T
        % t is the time of this constraint
        p = x1 + vec * t;
        for j = 0 : n-1 
            if j >= ki
                product = 1;
                for m = 0 : ki-1
                    product = product * (j-m);
                end
                A(j+1)= product*t^(j-ki);
            else
                A(j+1)=0;
            end
        end

        % check left/up
        
        if x_coord
           p_left = p(1);
           while (collisionChecker.isFree([p_left, p(2)]) && p_left >= 0)
               p_left = p_left - step;
           end
           b = p_left;
           Ai = [Ai; -A];
           bi = [bi; -b];
        else
           p_up = p(2);
           while (collisionChecker.isFree([p(1), p_up]) && p_up <= 1)
               p_up = p_up + step;
           end
           b = p_up;
           Ai = [Ai; A];
           bi = [bi; b];
        end

        % check right/down
        
        if x_coord
           p_right = p(1);
           while (collisionChecker.isFree([p_right, p(2)]) && p_right <= 1)
               p_right = p_right + step;
           end
           b = p_right;
           Ai = [Ai; A];
           bi = [bi; b];
        else
           p_down = p(2);
           while (collisionChecker.isFree([p(1), p_down]) && p_down >= 0)
               p_down = p_down - step;
           end
           b = p_down;
           Ai = [Ai; -A];
           bi = [bi; -b];
        end
        
    end
    
    % Ai=[];
    % bi=[];

    options = optimoptions('quadprog', 'Algorithm', 'interior-point-convex');
    p = quadprog(H,zeros(length(H),1),Ai,bi,Ae,be,[],[],[],options);

    t = 0: dt : T;
    traj = polyval(flip(p), t);
    traj = traj';

end