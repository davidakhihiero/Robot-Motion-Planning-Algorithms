function cf = collisionFree(collisionChecker, u, v, step)
    % function to check if the line between nodes u and v is collision free
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