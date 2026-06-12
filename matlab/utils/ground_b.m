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
