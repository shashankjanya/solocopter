function drawaircraft(uu)
    pn = uu(1);
    pe = uu(2);
    pd = uu(3);
    u = uu(4);
    v = uu(5);
    w = uu(6);
    phi = uu(7);
    theta = uu(8);
    psi = uu(9);
    p = uu(10);
    q = uu(11);
    r = uu(12);
    t = uu(13);

    persistent aircraft_handle
    persistent vertices
    persistent faces
    persistent facecolors
    
    if t == 0 
        figure(1), clf
        [vertices, faces, facecolors] = definemonocopter();
        
        aircraft_handle = sketchaircraft(vertices, faces, facecolors, ...
                                  pn, pe, pd, phi, theta, psi, []);
        
        hold on
        axis_length = 5;
        width = 2;       
     
        plot3([0, axis_length], [0, 0], [0, 0], 'g', 'LineWidth', width);
        text(axis_length, 0, 0, 'E');

        plot3([0, 0], [0, axis_length], [0, 0], 'r', 'LineWidth', width);
        text(0, axis_length, 0, 'N');
        
        plot3([0, 0], [0, 0], [0, -axis_length], 'b', 'LineWidth', width);
        text(0, 0, axis_length, 'D');

        % xlabel('North')
        % ylabel('East')
        % zlabel('Down')
        view(32, 47) 
        axis equal;
        grid on
        box on
        
        limit = 1;
        axis([-limit, limit, -limit, limit, -limit, limit]);
            
    else
        sketchaircraft(vertices, faces, facecolors, ...
                       pn, pe, pd, phi, theta, psi, aircraft_handle);
    end
end

function handel = sketchaircraft(v, f, fc, ...
                        pn, pe, pd, phi, theta, psi, ...
                        handel)
    v = rotate(v', phi, theta, psi);
    % v = translate(v, pn, pe, pd);
    v = v';
    R =  [0, 1, 0;
          1, 0, 0;
          0, 0, -1];
    v = v*R;

    if isempty(handel)
        handel = patch('Vertices', v, 'Faces', f,...
                 'FaceVertexCData', fc,...
                 'FaceColor','flat',...
                 'EraseMode', 'normal');
    else 
        set(handel, 'Vertices', v, 'Faces', f)
        drawnow
    end
end

function xyz = rotate(xyz, phi, theta,  psi)
    R_psi = [cos(psi), sin(psi), 0;
            -sin(psi), cos(psi), 0
            0, 0, 1];

    R_theta = [cos(theta), 0, -sin(theta);
                0 , 1, 0;
                sin(theta), 0, cos(theta)];

    R_phi = [1, 0, 0;
             0, cos(phi), sin(phi);
             0, -sin(phi), cos(phi)];

    R = R_phi*R_theta*R_psi;
    xyz = R'*xyz;
end

function xyz = translate(xyz, pn, pe, pd)
    xyz = xyz + repmat([pn; pe; pd], 1, size(xyz, 2));
end

function [v, f, fc] = definemonocopter()
    radius = 0.6;    
    height = 0.6;    
    num_sides = 8;   

    angles = linspace(0, 2*pi, num_sides + 1);
    angles(end) = []; 
    
    % Preallocate vertex array (18 vertices total)
    % 1-8: Top Ring
    % 9-16: Bottom Ring
    % 17: Top Center
    % 18: Bottom Center
    v = zeros(2 * num_sides + 2, 3); 

    % Generate Top Ring (Vertices 1 to 8)
    for i = 1:num_sides
        v(i, :) = [radius * cos(angles(i)), radius * sin(angles(i)), height/2];
    end

    % Generate Bottom Ring (Vertices 9 to 16)
    for i = 1:num_sides
        v(i + num_sides, :) = [radius * cos(angles(i)), radius * sin(angles(i)), -height/2];
    end

    % Center Points
    v(17, :) = [0, 0, height/2];  % Top Center
    v(18, :) = [0, 0, -height/2]; % Bottom Center

    % --- 2. Define Faces (f) ---
    % We define faces using triangles [v1, v2, v3]
    
    f = [];
    
    % -- Top Surface (Fan pattern connecting to Top Center) --
    % Faces 1-8
    for i = 1:num_sides
        next_i = mod(i, num_sides) + 1;
        % Triangle: Center(17) -> Current Rim -> Next Rim
        f = [f; 17, i, next_i]; 
    end

    % -- Bottom Surface (Fan pattern connecting to Bottom Center) --
    % Faces 9-16
    for i = 1:num_sides
        % Offset indices by num_sides to get bottom ring
        curr_i = i + num_sides; 
        next_i = mod(i, num_sides) + 1 + num_sides;
        % Triangle: Center(18) -> Next Rim -> Current Rim (Reversed for correct normal)
        f = [f; 18, next_i, curr_i];
    end

    % -- Side Surfaces (Connecting Top Ring to Bottom Ring) --
    % Each side is a rectangle, made of 2 triangles.
    % Faces 17-32
    for i = 1:num_sides
        top_curr = i;
        top_next = mod(i, num_sides) + 1;
        bot_curr = i + num_sides;
        bot_next = mod(i, num_sides) + 1 + num_sides;
        
        % Triangle 1
        f = [f; top_curr, bot_curr, top_next];
        % Triangle 2
        f = [f; top_next, bot_curr, bot_next];
    end

    % --- 3. Define Colors (fc) ---
    myred = [1, 0, 0]; 
    myblue = [0, 1, 0];   
    mydark = [0.2, 0.2, 0.2];

    fc = [];

    % Colors for Top Surface (8 faces)
    for i = 1:num_sides
        fc = [fc; myred];
    end

    % Colors for Bottom Surface (8 faces)
    for i = 1:num_sides
        fc = [fc; myblue];
    end

    % Colors for Sides (16 faces - 2 per side)
    for i = 1:(num_sides * 2)
        fc = [fc; mydark];
    end
    
    % --- 4. Define Body Axes (Arrows) ---
    % Add X, Y, Z axes as thin boxes
    axis_len = radius * 1.5;
    w = 0.02; 
    
    % Generic Box Vertices (Aligned along +X axis)
    % 8 corners of a box from x=0 to x=axis_len
    box_v = [
        0, -w, -w;        % 1
        axis_len, -w, -w; % 2
        axis_len,  w, -w; % 3
        0,  w, -w;        % 4
        0, -w,  w;        % 5
        axis_len, -w,  w; % 6
        axis_len,  w,  w; % 7
        0,  w,  w;        % 8
    ];

    % Box Faces (12 triangles)
    box_f = [
        1, 2, 3; 1, 3, 4; % Bottom
        5, 6, 7; 5, 7, 8; % Top
        1, 2, 6; 1, 6, 5; % Side 1
        2, 3, 7; 2, 7, 6; % End
        3, 4, 8; 3, 8, 7; % Side 2
        4, 1, 5; 4, 5, 8; % Start
    ];

    % -- X Axis (Red) --
    % Vertices are already along X
    start_idx = size(v, 1);
    v = [v; box_v];
    f = [f; box_f + start_idx];
    fc = [fc; repmat([1, 0, 0], 12, 1)]; % Red

    % -- Y Axis (Green) --
    % Swap X and Y coordinates to align along Y
    % Use width (col 2) for X, length (col 1) for Y, width (col 3) for Z
    box_v_y = [box_v(:,2), box_v(:,1), box_v(:,3)];
    start_idx = size(v, 1);
    v = [v; box_v_y];
    f = [f; box_f + start_idx];
    fc = [fc; repmat([0, 1, 0], 12, 1)]; % Green

    % -- Z Axis (Blue) --
    % Swap X and Z coordinates to align along Z
    % Use width (col 2) for X, width (col 3) for Y, length (col 1) for Z
    box_v_z = [box_v(:,2), box_v(:,3), box_v(:,1)];
    start_idx = size(v, 1);
    v = [v; box_v_z];
    f = [f; box_f + start_idx];
    fc = [fc; repmat([0, 0, 1], 12, 1)]; % Blue

end