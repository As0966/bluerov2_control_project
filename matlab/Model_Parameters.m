function params = Model_Parameters()
% Model_Parameters - Physical parameters for BlueROV2 (4-DOF)
% Based on the HASTEN-AUV Project specifications.

    % Mass and Moments of Inertia
    params.m = 11.5; % [kg]
    params.Iz = 0.16; % [kg.m^2]
    
    % Added Mass (Simplified for 4-DOF)
    params.X_u_dot = 6.3567;
    params.Y_v_dot = 7.1206;
    params.Z_w_dot = 12.6849;
    params.N_r_dot = 0.1264;
    
    % Linear Damping
    params.X_u = -4.03;
    params.Y_v = -6.22;
    params.Z_w = -5.18;
    params.N_r = -0.07;
    
    % Quadratic Damping
    params.X_uu = -18.18;
    params.Y_vv = -21.66;
    params.Z_ww = -36.99;
    params.N_rr = -1.55;
    
    % Buoyancy and Gravity
    params.W = params.m * 9.81;
    params.B = params.W + 2.0; % Slightly positively buoyant
    
    % Center of Buoyancy / Center of Gravity (Inertial Frame offsets)
    params.zg = 0.02; % [m]
    params.zb = -0.05; % [m]
    
    fprintf('Model Parameters Loaded Successfully for 4-DOF Analysis.\n');
end
