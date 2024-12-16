classdef BinaryHeap
    properties
        root_node = [];
        heap = {};
        n_nodes = 0;
    end

    methods
        function obj = BinaryHeap(root_node, size_of_heap)
            obj.root_node = root_node;
            obj.heap = cell(1, size_of_heap); 
            obj.heap{1} = root_node;
            obj.n_nodes = 1;
        end

        function is_empty = isEmpty(obj)
            is_empty = obj.n_nodes == 0;
        end

        function parent_index = getParent(~, index)
            parent_index = floor(index / 2);
        end

        function left_child_index = getLeftChild(obj, index)
            left_child_index = index * 2;
            if left_child_index > obj.n_nodes
                left_child_index = 0;
            end
        end

        function right_child_index = getRightChild(obj, index)
            right_child_index = index * 2 + 1;
            if right_child_index > obj.n_nodes
                right_child_index = 0;
            end
        end

        function obj = swap(obj, index1, index2)
            temp = obj.heap{index1};
            obj.heap{index1} = obj.heap{index2};
            obj.heap{index2} = temp;
        end

        function obj = insertNode(obj, node)
            obj.n_nodes = obj.n_nodes + 1;
            index = obj.n_nodes;
            obj.heap{index} = node;

            if obj.n_nodes == 1
                return;
            end
            parent_index = obj.getParent(index);
            parent_cost = obj.heap{parent_index}.cost;

            while parent_index >= 1 && parent_cost > node.cost
                obj = obj.swap(index, parent_index);
                index = parent_index;
                parent_index = obj.getParent(index);
                if parent_index < 1
                    break;
                end
                parent_cost = obj.heap{parent_index}.cost;
            end

        end

        function [obj, next] = getNextNodeInPQ(obj)
            index = 1;
            next = obj.heap{index};
            obj.heap{index} = obj.heap{obj.n_nodes};
            node = obj.heap{index};
            obj.n_nodes = obj.n_nodes - 1;

            left_child_index = obj.getLeftChild(index);

            if left_child_index ~= 0
                left_child_cost = obj.heap{left_child_index}.cost;
            else
                left_child_cost = Inf;
            end

            right_child_index = obj.getRightChild(index);
            if right_child_index ~= 0
                right_child_cost = obj.heap{right_child_index}.cost;
            else
                right_child_cost = Inf;
            end
  

            while left_child_cost < node.cost || right_child_cost < node.cost
                if left_child_cost < right_child_cost
                    obj = obj.swap(index, left_child_index);
                    index = left_child_index;
                    node = obj.heap{index};

                    left_child_index = obj.getLeftChild(index);
                    if left_child_index ~= 0
                        left_child_cost = obj.heap{left_child_index}.cost;
                    else
                        left_child_cost = Inf;
                    end
                    right_child_index = obj.getRightChild(index);
                    if right_child_index ~= 0
                        right_child_cost = obj.heap{right_child_index}.cost;
                    else
                        right_child_cost = Inf;
                    end
                else
                    obj = obj.swap(index, right_child_index);
                    index = right_child_index;
                    node = obj.heap{index};

                    left_child_index = obj.getLeftChild(index);
                    if left_child_index ~= 0
                        left_child_cost = obj.heap{left_child_index}.cost;
                    else
                        left_child_cost = Inf;
                    end
                    right_child_index = obj.getRightChild(index);
                    if right_child_index ~= 0
                        right_child_cost = obj.heap{right_child_index}.cost;
                    else
                        right_child_cost = Inf;
                    end
                end
            end
        end

    end

end