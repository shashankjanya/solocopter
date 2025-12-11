function sys = dynamics_six_dof(x, uu, MAV)
    pn = x(1);
    pe = x(2);
    pd = x(3);
    u = x(4);
    v = x(5);
    w = x(6);
    e0 = x(7);
    e1 = x(8);
    e2 = x(9);
    e3 = x(10);
    p = x(11);
    q = x(12);
    r = x(13);

    fx = uu(1);
    fy = uu(2);
    fz = uu(3);
    l = uu(4);
    m = uu(5);
    n = uu(6);

    mass = MAV.mass;
    
    Rb2i = rotate_body2inertial(e0, e1, e2, e3);
    posdot = Rb2i * [u; v; w];

    udot = r*v - q*w + fx/mass;
    vdot = p*w - r*u + fy/mass;
    wdot = q*u - p*v + fz/mass;

    Rkin = quaternions_kinematics(p, q, r, e0, e1, e2, e3, 10);
    quatdot = Rkin * [e0; e1; e2; e3];

    [L1, L2, L3, L4, L5, L6, L7, L8] = inertia_terms(MAV);
    
    pdot = L1*p*q - L2*q*r + L3*l + L4*n;
    qdot = L5*p*r - L6*(p^2 - r^2) + (1/MAV.Iyy)*m;
    rdot = L7*p*q - L1*q*r + L4*l + L8*n;
    
    % disp(size(posdot))
    % disp(size([udot; vdot; wdot]))
    % disp(size(quatdot))
    % disp(size([pdot; qdot; rdot]))
  
    sys = [posdot;...
           udot; vdot; wdot;...
           quatdot;...
           pdot; qdot; rdot];
end

function R =  rotate_body2inertial(e0, e1, e2, e3)
    R = zeros(3,3);
    R(1,1) = e0^2 + e1^2 - e2^2 - e3^2;
    R(1,2) = 2*(e1*e2 - e0*e3);
    R(1,3) = 2*(e1*e3 + e0*e2);
    R(2,1) = 2*(e1*e2 + e0*e3);
    R(2,2) = e0^2 - e1^2 + e2^2 - e3^2;
    R(2,3) = 2*(e2*e3 - e0*e1);
    R(3,1) = 2*(e1*e3 - e0*e2);
    R(3,2) = 2*(e2*e3 + e0*e1);
    R(3,3) = e0^2 - e1^2 - e2^2 + e3^2;    
end

function kin_mat = quaternions_kinematics(p, q, r, e0, e1, e2, e3, lambda)
    mat = [0, -p, -q, -r;
           p, 0, r, -q;
           q, -r, 0, p;
           r, q, -p, 0];
    mod_e = norm([e0, e1, e2, e3], 2);
    norm_mat = (lambda*(1 - mod_e)).* eye(4,4);

    kin_mat = 0.5 .* (mat + norm_mat);
end

function [L1, L2, L3, L4, L5, L6, L7, L8] = inertia_terms(MAV)
    Ix = MAV.Ixx;
    Iy = MAV.Iyy;
    Iz = MAV.Izz;
    Ixz = MAV.Ixz;

    L = Ix*Iz - Ixz^2;
    L1 = Ixz*(Ix - Iy + Iz) / L;
    L2 = (Iz*(Iz - Iy) + Ixz^2) / L;
    L3 = Iz / L;
    L4 = Ixz / L;
    L5 = (Iz - Ix) / Iy;
    L6 = Ixz / Iy;
    L7 = ((Ix - Iy)*Ix + Ixz^2) / L;
    L8 = Ix / L;
end