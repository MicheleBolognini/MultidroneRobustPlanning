function displayMesh(mesh, color)
if ~exist("color","var")
    color = [0.4 0.8 0.8];
end
    patch(mesh,'FaceColor',       color, ...
        'EdgeColor',       [0.0 0.0 0.0],        ...
        'FaceLighting',    'gouraud',     ...
        'AmbientStrength', 0.25);
    % Add a camera light, and tone down the specular highlighting
    camlight('left');
    material('dull');
    % Fix the axes scaling, and set a nice view angle
    axis('image');
    view([-135 35]);
    xlabel("X [m]"), ylabel("Y [m]"), zlabel("Z [m]")
    grid on
end