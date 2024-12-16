function [] = plotGoalRegion(fig, center, r_goal)
    figure(fig);
    hold on;

    theta = 0:0.5:2*pi;
    x = r_goal * cos(theta) + center(1);
    y = r_goal * sin(theta) + center(2);

    scatter(x, y, 2, 'r');

end