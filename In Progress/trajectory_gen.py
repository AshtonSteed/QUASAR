import numpy as np

def generate_trajectory(t, shape_id, width, height, speed, t_total):
    # Initialize arrays
    p = np.zeros(3)
    v = np.zeros(3)
    a = np.zeros(3)
    yaw_des = 0.0
    
    # Hold final position if time exceeds total time
    if t > t_total:
        t = t_total
        
    if shape_id == 1:  # HELIX
        radius = width / 2.0
        omega = speed
        v_z = height / t_total
        
        p[0] = radius * np.cos(omega * t)
        p[1] = radius * np.sin(omega * t)
        p[2] = v_z * t
        
        v[0] = -radius * omega * np.sin(omega * t)
        v[1] = radius * omega * np.cos(omega * t)
        v[2] = v_z
        
        yaw_des = omega * t
        
    elif shape_id == 2:  # FIGURE-8
        A = width / 2.0
        wx = speed
        wy = 2.0 * speed
        
        p[0] = A * np.sin(wx * t)
        p[1] = A * np.sin(wy * t)
        p[2] = height
        
        v[0] = A * wx * np.cos(wx * t)
        v[1] = A * wy * np.cos(wy * t)
        v[2] = 0.0
        
        yaw_des = np.arctan2(v[1], v[0])
        
    elif shape_id == 3:  # SQUARE
        leg_time = t_total / 4.0
        v_xy = width / leg_time
        p[2] = height
        
        if t <= leg_time:
            p[0] = v_xy * t
            v[0] = v_xy
        elif t <= 2 * leg_time:
            p[0] = width
            p[1] = v_xy * (t - leg_time)
            v[1] = v_xy
        elif t <= 3 * leg_time:
            p[0] = width - (v_xy * (t - 2 * leg_time))
            p[1] = width
            v[0] = -v_xy
        else:
            p[0] = 0.0
            p[1] = width - (v_xy * (t - 3 * leg_time))
            v[1] = -v_xy
            
        yaw_des = np.arctan2(v[1], v[0])
        
    return p, v, a, yaw_des