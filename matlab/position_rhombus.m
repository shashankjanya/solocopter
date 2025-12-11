function pos_cmd = position_rhombus(t)
    center = [0, 0, -20, 0]; 

    width_x  = 10; 
    width_y  = 10;
    height_z = 0;
    
    % durations = [3, 10, 10, 10, 10]; 
    durations = [10, 15, 10, 15, 10, 15, 10, 15, 10, 15];

    P0 = center;
    P1 = center + [width_x, 0, -5, 0];
    P2_bar = P1 + [0, 0, 0, deg2rad(90+45)];
    P2 = center + [0, width_y, 0, deg2rad(90+45)];
    P3_bar = P2 + [0, 0, 0, -deg2rad(90+45)];
    P3 = center + [-width_x, 0, +5, 0];
    P4_bar = P3 + [0, 0, 0, 0];
    P4 = center + [0, -width_y, 0, 0];
    P5_bar = P4 + [0, 0, 0, deg2rad(90)];
    P5 = center + [0, 0, 0, deg2rad(90)];

    % Waypoints = [P0; 
    %              P1; 
    %              P2; 
    %              P3; 
    %              P4];

    Waypoints = [P0; 
                 P1; 
                 P2_bar;
                 P2;
                 P3_bar;
                 P3;
                 P4_bar;
                 P4;
                 P5_bar;
                 P5];

    time_bounds = cumsum(durations); 
    total_cycle = time_bounds(end);

    local_t = mod(t, total_cycle);

    idx = length(durations); 
    
    for i = 1:length(time_bounds)
        if local_t < time_bounds(i)
            idx = i;
            break; 
        end
    end
    pos_cmd = Waypoints(idx, :);

end