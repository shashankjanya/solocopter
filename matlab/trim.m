function u_trim = trim(MAV)
    % initial guess
    % hover condition
    x0 = [-deg2rad(5), +deg2rad(5), +deg2rad(5), -deg2rad(5), +0.68];
    % ground condition
    % x0 = [-deg2rad(0), +deg2rad(0), +deg2rad(0), -deg2rad(0), 0];

    flap_limit = deg2rad(35);
    lb = [-flap_limit, -flap_limit, -flap_limit, -flap_limit, 0];
    ub = [ flap_limit,  flap_limit,  flap_limit,  flap_limit, 1];

    % options = optimset('Display', 'iter', 'TolFun', 1e-6);
    % [u_trim, cost] = fminsearch(@(x) trim_cost(x, MAV), x0, options);
    
    options = optimoptions('fmincon', 'Display','iter', 'Algorithm','sqp');
    [u_trim, cost] = fmincon(@(x) trim_cost(x, MAV), x0, [], [], [], [], ...
                             lb, ub, [], options);

    fprintf('--- TRIM RESULTS ---\n');
    fprintf('Trim Throttle: %f\n', u_trim(5));
    fprintf('Trim Flap Angle: %f rad\n', u_trim(1:4));
    fprintf('cost %f\n', cost);
    
end

function cost = trim_cost(uu, MAV)
    % trimmed state variables
    pn = 0; pe = 0; pd = MAV.pd0;
    u = 0; v = 0; w = 0;
    e0 = 1; e1 = 0; e2 = 0; e3 = 0;
    p = 0; q = 0; r = 0;

    x = [pn, pe, pd, u, v, w, e0, e1, e2, e3, p, q, r];
    x_cont = [pn, pe, pd, u, v, w, 0, 0, 0, p, q, r];

    wind = zeros(6, 1);
    out_cont = force_moment(x_cont, uu, wind, MAV);
    out = dynamics_six_dof(x, [out_cont(1:3); out_cont(4:6)], MAV);
    
    w_acc = 1;
    w_ang_acc = 1;
    cost = w_acc*sum(out(4:6).^2) + w_ang_acc*sum(out(11:13).^2);
end