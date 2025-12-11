%% Initial states
MAV.pn0 = 0;
MAV.pe0 = 0;
MAV.pd0 = -20;
MAV.u0 = 0;
MAV.v0 = 0;
MAV.w0 = 0;
MAV.e00 = 1;
MAV.e10 = 0;
MAV.e20 = 0;
MAV.e30 = 0;
MAV.p0 = 0;
MAV.q0 = 0;
MAV.r0 = 0;

%% Constants
MAV.rho = 1.225;
MAV.g = 9.81;

%% MAV inertial and geometric parameters
MAV.mass = 0.8;
MAV.c = 0.19;
MAV.b = 2.9;
MAV.e = 0.9;
MAV.Ixx = 0.824;
MAV.Iyy = 1.135;
MAV.Izz = 1.759;
MAV.Ixz = 0.0;

%% Propulser parameters
MAV.prop.d_inch = 11;
MAV.prop_data.volt_max = 14.4;
MAV.prop_data.d = MAV.prop.d_inch * 2.54 * 1e-2;
MAV.prop_data.prop_area = pi * ((MAV.prop.d_inch * 2.54 * 1e-2)^2) / 4;
MAV.prop.Kv_star = 1250;
MAV.prop_data.Kv = MAV.prop.Kv_star * (2*pi / 60);
MAV.prop_data.Kq = 60 / (2*pi*MAV.prop.Kv_star) ;
MAV.prop_data.R = 52e-3;
MAV.prop_data.i0 = 1.3;
MAV.prop.tc = 0.1;
MAV.prop.Iprop = 6.5e-5;


MAV.prop_data.quad.theta1 = [0.0924345  -0.03558534 -0.24652247]; 
MAV.prop_data.quad.theta2 = [0.02901829  0.06356233 -0.18441092];
MAV.prop_data.ford.theta1 = [0.08865825 -0.05833572  0.56924846 -3.85473642  5.0051231];
MAV.prop_data.ford.theta2 = [0.03610616 -0.09639692  1.01153022 -3.55956     3.62941509];

%% Trim controls
% Hover state
MAV.trim.del1 = -0.304322715939828;
MAV.trim.del2 = 0.304322650187243;
MAV.trim.del3 = 0.304322717152923;
MAV.trim.del4 = -0.304322122478042;
MAV.trim.delt = 0.721324091444547;

% On-Ground state
% MAV.trim.del1 = 0;
% MAV.trim.del2 = 0;
% MAV.trim.del3 = 0;
% MAV.trim.del4 = 0;
% MAV.trim.delt = 0;

%% Ground force constants
MAV.ground.damp = 100;
MAV.ground.stiffness = 3100;
MAV.ground.vd = 50;
MAV.ground.fric_coeff = 0.5;
MAV.ground.damp_rot = 1;
MAV.ground.stiffness_rot = 2;
MAV.ground.vd_rot = 5;
MAV.ground.fric_coeff_rot = 0.025;

%% Flap parameters
MAV.flap.cla = 2*pi;
MAV.flap.cdo = 0.03;
MAV.flap.k = 1/(pi*0.8*4);
MAV.flap.s = 0.15*0.075;
MAV.flap.xt = 0.25; 
MAV.flap.yt = 0.25; 
MAV.flap.zt = 0.25;
MAV.flap.tc = 0.1;

%% Wind parameters
SIM.wind.wn_s = 0;
SIM.wind.we_s = 0;
SIM.wind.wd_s = 0;

%% Simulation settings 
sample_time = 0.005;
SIM.ref_lon = 8.545594;
SIM.ref_lat = 47.397742;
SIM.ref_height = 8.545594;

%% Controller Inputs
% MAV_CONT.trim.taux = 0;
% MAV_CONT.trim.tauy = 0;
% MAV_CONT.trim.tauz = 0.1423e-4;
% MAV_CONT.trim.t = 7.848;

Simulink.Bus.createObject(MAV);
MAV_Bus = slBus1;