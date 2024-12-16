function h = circle(x, y, r, th, style, lw)
    hold on
    % th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
    h = plot(xunit, yunit, style, LineWidth=lw);
end