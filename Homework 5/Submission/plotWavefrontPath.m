function [p] = plotWavefrontPath(fig, resolution, env_size, nodes_along_path, style)
    p = [];
    figure(fig);
    axis([0 env_size 0 env_size]);

    xticks(0:0.1:env_size); 
    yticks(0:0.1:env_size); 
    
    xticklabels(0:0.1:env_size);
    yticklabels(0:0.1:env_size);
    hold on;
    for i = 1:size(nodes_along_path, 1)-1
        V1 = (nodes_along_path(i, :)) * resolution;
        V1(2) = env_size - V1(2);
        V2 = (nodes_along_path(i+1, :)) * resolution;
        V2(2) = env_size - V2(2);


        p = plot([V1(1), V2(1)], [V1(2), V2(2)], style, 'LineWidth', 3);

    end
    hold off;
end