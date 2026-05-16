import numpy as np

def reference_trajectory(t, shape='spline', T_final=60.0):
    """
    High-fidelity trajectory generator starting at (0,0) for BlueROV2.
    """
    omega_base = 2.0 * np.pi / T_final
    omega_2 = 4.0 * np.pi / T_final # 2 cycles

    if shape == 'circle':
        R = 3.0
        # Starts at (0,0) and does a circle
        x_d = R * np.sin(omega_2 * t)
        y_d = R * (1.0 - np.cos(omega_2 * t))
        z_d = -3.0
        
        x_dot_d = R * omega_2 * np.cos(omega_2 * t)
        y_dot_d = R * omega_2 * np.sin(omega_2 * t)
        z_dot_d = 0.0
        
        x_ddot_d = -R * omega_2**2 * np.sin(omega_2 * t)
        y_ddot_d = R * omega_2**2 * np.cos(omega_2 * t)
        z_ddot_d = 0.0

    elif shape == 'figure8':
        # Lissajous 8 starting at (0,0): x = a*sin(wt), y = b*sin(2wt)
        a = 4.0
        b = 2.0
        omega = 2.0 * np.pi / T_final # 1 lap in T_final (60s)
        
        x_d = a * np.sin(omega * t)
        y_d = b * np.sin(2.0 * omega * t)
        z_d = -3.0
        
        x_dot_d = a * omega * np.cos(omega * t)
        y_dot_d = 2.0 * b * omega * np.cos(2.0 * omega * t)
        z_dot_d = 0.0
        
        x_ddot_d = -a * omega**2 * np.sin(omega * t)
        y_ddot_d = -4.0 * b * omega**2 * np.sin(2.0 * omega * t)
        z_ddot_d = 0.0

    else: # Spline
        # Linear X, sinusoidal Y/Z
        v_x = 10.0 / T_final
        x_d = v_x * t
        y_d = 2.0 * np.sin(omega_2 * t)
        z_d = -3.0 + 0.5 * (1.0 - np.cos(omega_base * t))
        
        x_dot_d = v_x
        y_dot_d = 2.0 * omega_2 * np.cos(omega_2 * t)
        z_dot_d = 0.5 * omega_base * np.sin(omega_base * t)
        
        x_ddot_d = 0.0
        y_ddot_d = -2.0 * omega_2**2 * np.sin(omega_2 * t)
        z_ddot_d = 0.5 * omega_base**2 * np.cos(omega_base * t)

    # Orientation (Tangent)
    psi_d = np.arctan2(y_dot_d, x_dot_d)
    
    # Analytical Yaw Rate
    # psi_dot = (x'y'' - y'x'') / (x'^2 + y'^2)
    denom = x_dot_d**2 + y_dot_d**2
    if denom < 1e-6:
        psi_dot_d = 0.0
    else:
        psi_dot_d = (x_dot_d * y_ddot_d - y_dot_d * x_ddot_d) / denom
    
    return {
        "eta_d": np.array([x_d, y_d, z_d, psi_d]),
        "eta_dot_d": np.array([x_dot_d, y_dot_d, z_dot_d, psi_dot_d]),
        "eta_ddot_d": np.array([x_ddot_d, y_ddot_d, z_ddot_d, 0.0]),
    }
