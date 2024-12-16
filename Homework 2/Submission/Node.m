classdef Node
    properties
        cost;
        idx;
        actual_cost = 0;
        back_pt;
    end

    methods
        function obj = Node(cost, idx, back_pt)
            obj.cost = cost;
            obj.idx = idx;
            obj.back_pt = back_pt;
        end
    end
end