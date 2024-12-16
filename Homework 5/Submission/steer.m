function x = steer(x1, x2, step_steer)
    vec = (x2 - x1);
    normvec = norm(vec);
    x_vec = vec/normvec;

    if normvec > step_steer
        x = x1 + x_vec * step_steer;
    else
        x = x2;
    end

end