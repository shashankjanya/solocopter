function out = force_moment(x, uu, wind, MAV)
    % states
    pd = x(3); 
    u = x(4);
    v = x(5);
    w = x(6);
    phi = x(7);
    theta = x(8);
    psi = x(9);
    p = x(10);
    q = x(11);
    r = x(12);

    % control inputs
    del1 = uu(1);
    del2 = uu(2);
    del3 = uu(3);
    del4 = uu(4);
    delt = uu(5);
    
    % wind components
    wn_s = wind(1);
    we_s = wind(2);
    wd_s = wind(3);
    u_wg = wind(4);
    v_wg = wind(5);
    w_wg = wind(6);
    
    % relative velocity
    Ri2b = rotate_inter2body(phi, theta, psi);
    wind_b = Ri2b * [wn_s; we_s; wd_s] + [u_wg; v_wg; w_wg];
    ua = u - wind_b(1);
    va = v - wind_b(2);
    wa = w - wind_b(3);
    wind_ned = Ri2b' * wind_b;
    
    % absaloute velocity
    % velocity_a = sqrt(ua^2 + va^2 + wa^2);
    velocity_a = w;

    % propulser force
    volt = MAV.prop_data.volt_max * delt;
    prop_model = 'prop_second_order';
    % prop_model = 'prop_fourth_order';

    if strcmp(prop_model,'prop_fourth_order')
        [T, tou, omega] = prop_fourth_order(velocity_a, volt, MAV);
    else
        [T, tou, omega] = prop_second_order(velocity_a, volt, MAV);
    end

    if T > 0
        v_induced = sqrt(T / (2 * MAV.rho * MAV.prop_data.prop_area));
    else
        v_induced = 0;
    end
    T = clip(T, 0, 13);
    
    % gyroscopic torque 
    M_gyro = (omega*MAV.prop.Iprop).*[q; -p; 0];
    
    % v_flap_effective = v_induced - wa;
    v_flap_effective = v_induced + max(0, -wa);
    if v_flap_effective < 0; v_flap_effective = 0; end

    % aerodynamic parameters of the flap 
    cla = MAV.flap.cla;
    q_bar = 0.5 * MAV.rho * abs(v_flap_effective) * v_flap_effective;
    cdo = MAV.flap.cdo;
    k = MAV.flap.k;
    s = MAV.flap.s;
    xt = MAV.flap.xt; yt = MAV.flap.yt; zt = MAV.flap.zt;
    
    L = @(del) q_bar*s*cla*del;
    D = @(del) q_bar*s*(cdo + k*((cla*del)^2));
    
    M1 = [-zt   0 -zt   0  0;
            0 -zt   0 -zt  0;
           xt -yt -xt  yt  1];
    M2 = [  0 yt  0 -yt  0;
          -xt  0 xt   0  0;
            0  0  0   0  0];
    
    % force and moment at the cg
    M = M1*[L(del1); L(del2); L(del3); L(del4); tou] + M2*[D(del1); D(del2); D(del3); D(del4); delt];
    F = [(-L(del2) - L(del4));
         (L(del1) + L(del3)); 
         (-T + D(del1) + D(del2) + D(del3) + D(del4))];
    
    %  ground forces 
    FM_ground = ground_b(pd, u, v, w, phi, theta, psi, p, q, r, MAV);

    out = [(F + gravity_b(phi, theta, MAV) + FM_ground(1:3, 1)); 
        (M + FM_ground(4:6, 1) + M_gyro); wind_ned];
end

function f = gravity_b(phi, theta, MAV)
   f = (MAV.mass * MAV.g) .* [-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)];
end

function R = rotate_inter2body(phi, theta, psi)
    R_psi = [cos(psi), sin(psi), 0;
            -sin(psi), cos(psi), 0
            0, 0, 1];
    R_theta = [cos(theta), 0, -sin(theta);
                0 , 1, 0;
                sin(theta), 0, cos(theta)];
    R_phi = [1, 0, 0;
             0, cos(phi), sin(phi);
             0, -sin(phi), cos(phi)];
    R = R_phi * R_theta * R_psi;
end

function out = ground_b(pz, u, v, w, phi, theta, psi, p, q, r, MAV)
    % normal force 
    if -pz > 0
        f_normal = 0;
        out = zeros(6,1);
        return
    else 
        stiff_term = MAV.ground.stiffness * pz;
        damp_term = MAV.ground.damp * w;
        f_normal = stiff_term + damp_term;
    end

    f_normal = clip(f_normal, 0, 10 * MAV.mass * MAV.g);
    
    % friction_force
    Ri2b = rotate_inter2body(phi, theta, psi);
    Rb2i = Ri2b';
    v_ned = Rb2i * [u; v; w];
    
    f_friction = tanh(v_ned(1:2, 1) .* (1/MAV.ground.vd)) .* MAV.ground.fric_coeff; 
    f_friction = f_normal * f_friction;
    f_friction = clip(f_friction, -20, 20);
    
    f_ground = [-1.*f_friction; -f_normal];
    f_ground_b = Ri2b * f_ground;  

    % moments 
    if f_normal > 0
        rp = clip([phi; theta], -20*pi/180, 20*pi/180);
        moment_xy = [p; q] * MAV.ground.damp_rot + rp * MAV.ground.stiffness_rot;
        moment_xy = clip(moment_xy, -0.1, 0.1);
    
        moment_z = tanh(r .* (1/MAV.ground.vd_rot)) .* MAV.ground.fric_coeff_rot; 
        moment_z = clip((moment_z * f_normal), -0.1, 0.1);
    
        moment_ground_b = [-1.*moment_xy; -moment_z];
    else 
        moment_ground_b = [0; 0; 0];
    end

    out = [f_ground_b; moment_ground_b];
end

function val = clip(val, min_val, max_val)
    val = max(min_val, min(val, max_val));
end