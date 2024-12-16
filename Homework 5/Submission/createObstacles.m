function obstacles = createObstacles(new_obstacles, dense)
    if ~new_obstacles
        if ~dense
            obstacles = load("obstacles_sparse.mat");
        else
            obstacles = load("obstacles_dense.mat");
        end
        obstacles = obstacles.obstacles;
        return;
    end

    % figure;
    % axis([0 env_size 0 env_size])
    obstacles = {};
    disp("Type 'Y' to create an obstacle or 'N' to stop");
    count = 1;
    prompt = "Create an obstacle? (Y/N) ";
    cmd = upper(input(prompt, "s"));
    if isempty(cmd)
        cmd = "Y";
    end

    while cmd=="Y" 
        obstacle = drawpolygon("Color", "r");
        title("Configuration Space");
        obstacles{count} = obstacle;
        if count > 1
            fprintf("%d obstacles created\n", count);
        else
            disp("1 obstacle created");
        end

        count = count + 1;
        prompt = "Create an obstacle? (Y/N) ";
        cmd = upper(input(prompt, "s"));
        if isempty(cmd)
            cmd = "N";
        end
    end

    %save obstacles_sparse obstacles;
    save obstacles_new obstacles;

end