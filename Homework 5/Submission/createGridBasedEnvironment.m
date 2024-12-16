function bitmap = createGridBasedEnvironment(obstacles, env_size, resolution)
    n_cells = floor(env_size / resolution); % number of cells in the grid
    bitmap = zeros(n_cells); % bitmap array
    collisionChecker = CollisionChecker(obstacles);

    for i = 0:n_cells-1
        for j = 0:n_cells-1
            % check if the four corner of a grid cell is a free
            % configuration point and if it's not, the grid cell is marked
            % occupied 
            q1 = [j * resolution, i * resolution]; 
            q2 = [(j + 1) * resolution, i * resolution]; 
            q3 = [j * resolution, (i+1) * resolution]; 
            q4 = [(j+1) * resolution, (i+1) * resolution]; 
            if ~collisionChecker.isFree(q1) || ~collisionChecker.isFree(q2) ...
                    || ~collisionChecker.isFree(q3) || ~collisionChecker.isFree(q4)
                bitmap(n_cells-i, j+1) = 1;
            end
        end
    end

    
end