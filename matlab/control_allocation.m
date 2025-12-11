function [deltas] = control_allocation(fm, MAV, prev_delt)
    % desired_forces_moments = [Mx; My; Mz; Fz]
    
    volt_prev = MAV.prop_data.volt_max * prev_delt;
    [T1, tau1] = prop_second_order(0, volt_prev, MAV);
    v_flap = sqrt(T1 / (2 * MAV.rho * MAV.prop_data.prop_area));
    
    q = 0.5 * MAV.rho * (v_flap^2); 
    C_lift = q * MAV.flap.s * MAV.flap.cla;
    
    K_T = T1 / prev_delt;
    K_Q = tau1 / prev_delt;

    % Mapping: [d1, d2, d3, d4, dt] -> [Mx, My, Mz, Fz]
    xt = MAV.flap.xt; yt = MAV.flap.yt; zt = MAV.flap.zt;
    
    B = [ ...
        (-zt*C_lift),  0,            (-zt*C_lift), 0,            0;
        0,             (-zt*C_lift), 0,            (-zt*C_lift), 0;
        (xt*C_lift),   (-yt*C_lift), (-xt*C_lift), (yt*C_lift),  K_Q;
        0,             0,            0,            0,            -K_T;
    ];
    
    % Pseudo Inverse
    P = pinv(B);
    
    % Raw Actuator Commands
    deltas = P * fm;
end