clc; clear; close all;


obstacles = createObstacles(); % obstacles are saved to a file

plotObstacles(obstacles);


% Check if clicked point is in free space
c = CollisionChecker();

while true
    q = ginput(1);
    
    if (c.isFree(q))
        fprintf("Configuration (%f, %f) is in the free space\n", q(1), q(2));
    else
        fprintf("Configuration (%f, %f) is not in the free space\n", q(1), q(2));
    end
end