function isfree = isFree(q)
    obstacles = load("obstacles.mat");
    obstacles = obstacles.obstacles;
    
    isfree = true;

    for i = 1:size(obstacles, 2)
        xv = obstacles{i}.Position(:,1);
        yv = obstacles{i}.Position(:,2);

        in = inpolygon(q(1), q(2), xv, yv);

        if in
            isfree = false;
            break;
        end
    end
end