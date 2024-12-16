function obstacles = plotObstacles(file)
    obstacles = load(file);
    obstacles = obstacles.obstacles;
    f = figure;
    title("Configuration Space");

    ax = axes(f);

    hold on;

    for i = 1:size(obstacles, 2)
        copy(obstacles{i}, ax);
    end

end
    