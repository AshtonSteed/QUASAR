function [p, v, a, yaw_des] = discreteTrajectory_gen(t)
    % --- HARDCODED FLIGHT PARAMETERS ---
    shape_id = 1;      % 1: Helix, 2: Fig8, 3: Square
    width    = 2.0;    % Radius or width (m)
    height   = 1.5;    % Hover altitude (m)
    speed    = 1.0;    % Speed modifier
    t_total  = 20.0;   % Total flight time (s)
    
    % --- WAYPOINT SETTINGS ---
    wp_interval = 1.0; % Time in seconds between each new waypoint

    % Discretize the time to create a "staircase" or zero-order hold effect
    t_discrete = floor(t / wp_interval) * wp_interval;

    % Cap the waypoint time at the end of the flight
    if t_discrete > t_total
        t_discrete = t_total; 
    end

    % Initialize outputs
    p = zeros(3,1); 
    v = zeros(3,1); 
    a = zeros(3,1); 
    yaw_des = 0;
    
    % CRITICAL CHANGE: 
    % Because the waypoint is physically stationary until it jumps to the next one,
    % target velocity (v) and target acceleration (a) are strictly zero. 
    % The drone must fly point-to-point using only Position Error.

    switch shape_id
        case 1 % HELIX
            radius = width / 2;
            omega = speed; 
            v_z = height / t_total;
            p = [radius * cos(omega * t_discrete); radius * sin(omega * t_discrete); v_z * t_discrete];
            yaw_des = omega * t_discrete; 
                 
        case 2 % FIGURE-8
            A = width / 2; 
            wx = speed; wy = 2 * speed; 
            p = [A * sin(wx * t_discrete); A * sin(wy * t_discrete); height]; 
            % Use the analytical tangent for heading, evaluated at the waypoint
            yaw_des = atan2(A * wy * cos(wy * t_discrete), A * wx * cos(wx * t_discrete));
                 
        case 3 % SQUARE
            leg_time = t_total / 4;
            v_xy = width / leg_time; 
            p(3) = height; 
            if t_discrete <= leg_time
                p(1) = v_xy * t_discrete; 
            elseif t_discrete <= 2*leg_time
                p(1) = width; p(2) = v_xy * (t_discrete - leg_time); 
            elseif t_discrete <= 3*leg_time
                p(1) = width - (v_xy * (t_discrete - 2*leg_time)); p(2) = width; 
            else
                p(1) = 0; p(2) = width - (v_xy * (t_discrete - 3*leg_time)); 
            end
            yaw_des = atan2(p(2), p(1)); % Point towards the origin for simplicity
    end
end