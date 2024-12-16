function [tangentPoints] = internalTangents(q1, q2, r, lower, plot_tangents, style)
    c1=[q1(1); q1(2)];
    c2=[q2(1); q2(2)];
    v=c2-c1;

    theta = acos(2*r/norm(v));
    alpha = atan2(v(2), v(1));
    
    vn1= [cos(theta+alpha); sin(theta+alpha)]*r;
    
    vn2= [-cos(theta-alpha); sin(theta-alpha)]*r;

    if lower && plot_tangents
        plot([c1(1)+vn1(1)], [c1(2)+vn1(2)], 'r*');
        plot([c2(1)-vn1(1)], [c2(2)-vn1(2)], 'r*');
        tangentPoints = [c1(1)+vn1(1), c1(2)+vn1(2); c2(1)-vn1(1), c2(2)-vn1(2)];
    elseif plot_tangents
        plot([c1(1)-vn2(1)], [c1(2)-vn2(2)], 'r*');
        plot([c2(1)+vn2(1)], [c2(2)+vn2(2)], 'r*');
        tangentPoints = [c1(1)-vn2(1), c1(2)-vn2(2); c2(1)+vn2(1), c2(2)+vn2(2)];
    elseif lower
        tangentPoints = [c1(1)+vn1(1), c1(2)+vn1(2); c2(1)-vn1(1), c2(2)-vn1(2)];
    else
        tangentPoints = [c1(1)-vn2(1), c1(2)-vn2(2); c2(1)+vn2(1), c2(2)+vn2(2)];
    end

    if lower && plot_tangents
        plot([c1(1)+vn1(1), c2(1)-vn1(1)],[c1(2)+vn1(2), c2(2)-vn1(2)], style);
    elseif plot_tangents
        plot([c1(1)-vn2(1), c2(1)+vn2(1)],[c1(2)-vn2(2), c2(2)+vn2(2)], style);
    end
end