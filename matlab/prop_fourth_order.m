function [thrust, torque, omega] = prop_fourth_order(va, volt, MAV)
    % FOURTH_ORDER_FIT Calculates thrust, torque and omega based on propeller coefficients
    
    theta1 = MAV.prop_data.ford.theta1;
    theta2 = MAV.prop_data.ford.theta2;
    
    rho = MAV.rho;
    d = MAV.prop_data.d;
    Kq = MAV.prop_data.Kq;
    Kv = MAV.prop_data.Kv;
    R = MAV.prop_data.R;
    io = MAV.prop_data.i0;

    a = ((rho * d^5) / ((2*pi)^2)) * theta2(1);
    b = ((rho * d^4) / (2*pi)) * theta2(2) * va + (Kq/(Kv*R));
    c = (rho * d^3 * theta2(3) * va^2) - ((Kq/R)*volt) + Kq*io;    
    d_coeff = ((rho * d^5) / ((2*pi)^2)) * (((2*pi*va)/d)^3) * theta2(4);
    e = ((rho * d^5) / ((2*pi)^2)) * (((2*pi*va)/d)^4) * theta2(5);
    
    poly_coeffs = [a, b, c, d_coeff, e];

    all_roots = roots(poly_coeffs);
    
    is_real = abs(imag(all_roots)) < 1e-8;
    real_only_roots = real(all_roots(is_real));
    
    real_pos_roots = real_only_roots(real_only_roots > 0);
    
    if isempty(real_pos_roots)
        error('No real positive roots found for Omega.');
    end
    omega = max(real_pos_roots);

    t1 = ((rho * d^4 * theta1(1)) / ((2 * pi)^2)) * (omega^2);
    t2 = ((rho * d^3 * theta1(2) * va) / (2 * pi)) * omega;
    t3 = (rho * d^2 * theta1(3) * va^2);
    t4 = ((rho * d^4) / ((2 * pi)^2)) * theta1(4) * (((2*pi*va)/d)^3) * (1/omega);
    t5 = ((rho * d^4) / ((2 * pi)^2)) * theta1(5) * (((2*pi*va)/d)^4) * (1/(omega^2));
    
    thrust = t1 + t2 + t3 + t4 + t5;

    q1 = ((rho * d^5 * theta2(1)) / ((2 * pi)^2)) * (omega^2);
    q2 = ((rho * d^4 * theta2(2) * va) / (2 * pi)) * omega;
    q3 = (rho * d^3 * theta2(3) * va^2);
    q4 = ((rho * d^5) / ((2 * pi)^2)) * theta2(4) * (((2*pi*va)/d)^3) * (1/omega);
    q5 = ((rho * d^5) / ((2 * pi)^2)) * theta2(5) * (((2*pi*va)/d)^4) * (1/(omega^2));
    
    torque = q1 + q2 + q3 + q4 + q5;

end