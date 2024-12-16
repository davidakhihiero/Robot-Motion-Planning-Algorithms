function filename = createObstacles()
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

    save obstacles obstacles;
    filename = "obstacles.mat";

    close all;
end