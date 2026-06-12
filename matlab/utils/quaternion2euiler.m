function [sys] = quaternion2euiler(x)
    n = norm([x(7); x(8); x(9); x(10)]);
    e0 = x(7)./n;
    e1 = x(8)./n;
    e2 = x(9)./n;
    e3 = x(10)./n;
    % disp(norm([e0, e1, e2, e3]));
    
    psi = atan2(2*(e0*e3 + e1*e2), (e0^2 + e1^2 - e2^2 - e3^2));
    theta = asin(2*(e0*e2 - e1*e3));
    % theta = atan2( 2*(e0*e2 - e1*e3), 1 - 2*(e2^2 + e3^2) );
    phi = atan2(2*(e0*e1 + e2*e3), (e0^2 + e3^2 - e1^2 - e2^2));
    
    sys = zeros(12, 1); 
    sys(1:6) = x(1:6);
    sys(7) = phi;
    sys(8) = theta;
    sys(9) = psi;
    sys(10:12) = x(11:13);
end