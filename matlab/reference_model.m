function [out] = reference_model(theta_cmd, prev_states, wn, zeta, Ts)
    %#codegen
    theta_k = prev_states(1:3, 1);
    omega_k = prev_states(4:6, 1);
    
    % Euler integration 
    alpha_ref = (wn^2 .* (theta_cmd - theta_k)) - ((2 * zeta * wn) .* omega_k);
    omega_next = omega_k + alpha_ref * Ts;
    theta_next = theta_k + omega_next * Ts; 

    out = [theta_next; omega_next; alpha_ref];
end