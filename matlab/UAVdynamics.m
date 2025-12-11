function [sys, x0, str, ts, simStateCompliance] = UAVdynamics(t, x, u, flag, MAV)
    switch flag
        case 0
            [sys, x0, str, ts, simStateCompliance] = initialize(MAV);
        case 1
            sys = derivatives(x, u, MAV);
        case 2
            sys = update(t, x, u);
        case 3 
            sys = outputs(t, x);
        case 4
            sys = gettimeofnextvarhit(t, x, u);
        case 9
            sys = terminate(t, x, u);
        otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    end
end

function [sys, x0, str, ts, simStateCompliance] = initialize(MAV)

    size = simsizes;
    size.NumContStates = 13;
    size.NumDiscStates = 0;
    size.NumInputs = 6;
    size.NumOutputs = 12;
    size.DirFeedthrough = 0;
    size.NumSampleTimes = 1;
    
    sys = simsizes(size);

    x0 = [MAV.pn0;
          MAV.pe0;
          MAV.pd0;
          MAV.u0;
          MAV.v0;
          MAV.w0;
          MAV.e00;
          MAV.e10;
          MAV.e20;
          MAV.e30;
          MAV.p0;
          MAV.q0;
          MAV.r0];

    str = [];

    ts = [0, 0];

    simStateCompliance = 'UnknownSimState';
end
 
function sys = derivatives(x, uu, MAV)
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
  
    sys = [posdot;...
           udot; vdot; wdot;...
           quatdot;...
           pdot; qdot; rdot];
end

function sys = update(t, x, u)
    sys = [];
end

function sys = outputs(t, x)
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

function sys = gettimeofnextvarhit(t, x, u)
    sampletime = 1;
    sys = t + sampletime;
end

function sys = terminate(t, x, u)
    sys = [];
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