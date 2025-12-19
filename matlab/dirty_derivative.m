function out = dirty_derivative(omega_gyro, prev_states, tau, Ts)
    %#codegen
    % Tustin (Trapezoidal) Derivative with Low Pass Filter
    % H(s) = s / (tau*s + 1)
    u_prev = prev_states(1:3, 1);
    y_prev = prev_states(4:6, 1);

    den = 2 * tau + Ts;
    c1 = 2 / den;
    c2 = (2 * tau - Ts) / den;
    
    alpha_meas = c1 .* (omega_gyro - u_prev) + c2 .* y_prev;
    out = [omega_gyro; alpha_meas];    
end