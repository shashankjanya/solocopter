function [fm_trim, deltas_trim] = trim_rate_controller(MAV)
    weight_force = MAV.mass * 9.81; 
    x0 = [0, 0, 0, -weight_force]; 
    
    max_moment = 0.5; % Nm
    max_force  = 1.1 * weight_force;
    lb = [-max_moment, -max_moment, -max_moment, -max_force];
    ub = [ max_moment,  max_moment,  max_moment,  0]; % Fz usually negative for lift in NED

    options = optimoptions('fmincon', 'Display','iter', 'Algorithm','sqp', ...
                           'StepTolerance', 1e-8, 'OptimalityTolerance', 1e-8);

    [fm_trim, cost] = fmincon(@(x) trim_cost(x, MAV), x0, [], [], [], [], ...
                             lb, ub, [], options);

    [~, deltas_trim] = trim_cost_calculation(fm_trim, MAV);
     
    fprintf('--- TRIM RESULTS ---\n');
    fprintf('Trim Forces/Moments: [Mx=%.3f, My=%.3f, Mz=%.3f, Fz=%.3f]\n', fm_trim);
    fprintf('Trim Throttle: %.4f\n', fm_trim(4));
    fprintf('Trim Flaps (deg): %.2f  %.2f  %.2f  %.2f\n', rad2deg(fm_trim(1:3)));
    fprintf('Cost: %e\n', cost);
    % fprintf('delta  %.2f  %.2f  %.2f  %.2f\n', deltas_trim(1:5));
end

function cost = trim_cost(fm_input, MAV)
    [cost, ~] = trim_cost_calculation(fm_input, MAV);
end

function [cost, deltas_final] = trim_cost_calculation(fm_input, MAV)
    
    % initial guess
    current_throttle = 0.68; 
    for k = 1:5
        deltas = control_allocation(fm_input(:), MAV, current_throttle);
        new_throttle = deltas(5);
        
        if abs(new_throttle - current_throttle) < 1e-4
            current_throttle = new_throttle;
            break;
        end
        current_throttle = new_throttle;
    end
    
    deltas_final = deltas;

    % State Variables (Hover condition)
    pn = 0; pe = 0; pd = MAV.pd0;
    u = 0; v = 0; w = 0;
    e0 = 1; e1 = 0; e2 = 0; e3 = 0; % Level attitude
    p = 0; q = 0; r = 0;
    
    x = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r];
    x_cont = [pn, pe, pd, u, v, w, 0, 0, 0, p, q, r]; 
    wind = zeros(6, 1);
    
    out_cont = force_moment(x_cont, deltas, wind, MAV);
    x_dot = dynamics_six_dof(x, [out_cont(1:3); out_cont(4:6)], MAV);
    
    w_acc = 1;
    w_ang_acc = 10;
    
    cost = w_acc*sum(x_dot(4:6).^2) + w_ang_acc*sum(x_dot(11:13).^2);
    
    if any(abs(deltas(1:4)) > deg2rad(35)) || deltas(5) > 1 || deltas(5) < 0
        cost = cost + 1000; 
    end
end