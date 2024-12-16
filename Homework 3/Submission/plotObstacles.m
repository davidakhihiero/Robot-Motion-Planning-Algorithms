function [] = plotObstacles(fig, obstacle_vertices)
    figure(fig);
    title("Configuration Space");

    ax = axes(fig);

    hold on;

    for i = 1:size(obstacle_vertices, 1)
        xv = obstacle_vertices{i, 1};
        yv = obstacle_vertices{i, 2};

        fill(xv, yv, 'k');
        
    end

end
    