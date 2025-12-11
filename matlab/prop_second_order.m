function [thrust, torque, omega] = prop_second_order(va, volt, MAV)
    % QUADRATIC_FIT Calculates thrust, torque and omega using a 2nd order fit
    
    theta1 = MAV.prop_data.quad.theta1;
    theta2 = MAV.prop_data.quad.theta2;
    
    rho = MAV.rho;
    d = MAV.prop_data.d;
    Kq = MAV.prop_data.Kq;
    Kv = MAV.prop_data.Kv;
    R = MAV.prop_data.R;
    io = MAV.prop_data.i0;
    
    a = ((rho * d^5) / ((2*pi)^2)) * theta2(1);
    b = ((rho * d^4) / (2*pi)) * theta2(2) * va + (Kq/(Kv*R));
    c = (rho * d^3 * theta2(3) * va^2) - ((Kq/R)*volt) + Kq*io;

    discriminant = b^2 - 4*a*c;
    
    if discriminant < 0
        warning('Discriminant is negative. Omega will be complex.');
    end

    omega = (-b + sqrt(discriminant)) / (2*a);

    t1 = ((rho * d^4 * theta1(1)) / ((2 * pi)^2)) * (omega^2);
    t2 = ((rho * d^3 * theta1(2) * va) / (2 * pi)) * omega;
    t3 = (rho * d^2 * theta1(3) * va^2);
    
    thrust = t1 + t2 + t3;

    q1 = ((rho * d^5 * theta2(1)) / ((2 * pi)^2)) * (omega^2);
    q2 = ((rho * d^4 * theta2(2) * va) / (2 * pi)) * omega;
    q3 = (rho * d^3 * theta2(3) * va^2);
    
    torque = q1 + q2 + q3;

end