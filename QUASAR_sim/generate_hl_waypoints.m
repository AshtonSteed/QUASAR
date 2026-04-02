function waypoints = generate_hl_waypoints(shape_id, width, height, t_total, num_points)
    % GENERATE_HL_WAYPOINTS Pre-computes a flight plan for the Crazyflie High-Level Commander.
    % Outputs an N x 5 matrix. Each row is: [X, Y, Z, Yaw (rad), Duration (sec)]
    
    % Initialize time array
    t = linspace(0, t_total, num_points)';
    dt = t(2) - t(1); % Time to travel between each generated point
    
    % Pre-allocate the output matrix (N rows, 5 columns)
    waypoints = zeros(num_points, 5);
    
    switch shape_id
        case 1 % HELIX 
            radius = width / 2;
            omega = (2 * pi) / t_total; % Complete one full rotation in t_total
            v_z = height / t_total;
            
            waypoints(:, 1) = radius * cos(omega * t);           % X
            waypoints(:, 2) = radius * sin(omega * t);           % Y
            waypoints(:, 3) = v_z * t;                           % Z
            waypoints(:, 4) = omega * t;                         % Yaw
            waypoints(:, 5) = dt;                                % Duration
                 
        case 2 % FIGURE-8 
            A = width / 2; 
            wx = (2 * pi) / t_total; 
            wy = 2 * wx; 
            
            waypoints(:, 1) = A * sin(wx * t);                   % X
            waypoints(:, 2) = A * sin(wy * t);                   % Y
            waypoints(:, 3) = height;                            % Z (Constant altitude)
            
            % Calculate yaw tangent to the curve
            dx = A * wx * cos(wx * t);
            dy = A * wy * cos(wy * t);
            waypoints(:, 4) = atan2(dy, dx);                     % Yaw
            waypoints(:, 5) = dt;                                % Duration
                 
        case 3 % SQUARE (Only 4 waypoints)
            % We override num_points here because the High-Level Commander 
            % is smart enough to fly a perfectly straight line between corners.
            leg_time = t_total / 4;
            
            % [X, Y, Z, Yaw, Duration]
            waypoints = [
                0,     0,     height, 0,        leg_time; % Go to Start/Hover
                width, 0,     height, 0,        leg_time; % Leg 1
                width, width, height, pi/2,     leg_time; % Leg 2
                0,     width, height, pi,       leg_time; % Leg 3
                0,     0,     height, -pi/2,    leg_time  % Leg 4 (Back to start)
            ];
    end
end