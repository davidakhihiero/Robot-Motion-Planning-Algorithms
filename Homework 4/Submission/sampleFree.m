function q = sampleFree(collisionChecker, env_size)

    q = rand(1, 2) * env_size;

    while ~collisionChecker.isFree(q)
        q = rand(1, 2) * env_size;
    end

    q(1, 3) = rand * 2*pi;

end