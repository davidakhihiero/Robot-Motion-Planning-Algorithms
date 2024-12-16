function [x] = steer(x1, x2, step_steer)
    vec = (x2(1:2) - x1(1:2));
    normvec = norm(vec);
    x_vec = vec/normvec;

    if normvec > step_steer
        x = x1(1:2) + x_vec * step_steer;
        x(3) = x2(3);
    else
        x = x2;
    end
    

end