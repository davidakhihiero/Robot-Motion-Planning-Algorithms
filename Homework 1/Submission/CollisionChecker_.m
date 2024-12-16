classdef CollisionChecker_
    properties
       obstacles;
    end
    methods
        function obj = CollisionChecker_()
            data = load('obstacles.mat');
            obj.obstacles = data.obstacles; 
        end
        function isfree = isFree(obj, q)
            isfree = true;
        
            for i = 1:size(obj.obstacles, 2)
                xv = obj.obstacles{i}.Position(:,1);
                yv = obj.obstacles{i}.Position(:,2);

                [in, on] = inpolygon(q(1), q(2), xv, yv);
        
                if in
                    isfree = false;
                    break;
                end
            end
        end
    end
end