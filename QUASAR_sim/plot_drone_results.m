% Clear previous plots
figure(1); clf; hold on; grid on; axis equal;

% 1. Extract Data from the Timeseries objects
time = p_out.Time;
pos = p_out.Data; % This is Nx3
quat = q_out.Data; % This is Nx4

% 2. Plot the 3D Trajectory Line
plot3(pos(:,1), pos(:,2), pos(:,3), 'b', 'LineWidth', 2);

% 3. Visualize Orientation (Quaternions)
% We will plot a small "drone frame" every 50 steps so the plot isn't cluttered
sample_rate = 50; 
scale = 0.2; % Length of the orientation axes

for i = 1:sample_rate:length(time)
    % Current Position
    origin = pos(i, :);
    
    % Current Quaternion
    q = quat(i, :);
    
    % Convert Quaternion to Rotation Matrix
    % (Standard formula matching our plant model)
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);
    R = [1 - 2*(qy^2 + qz^2),   2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy);
         2*(qx*qy + qw*qz),     1 - 2*(qx^2 + qz^2), 2*(qy*qz - qw*qx);
         2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),   1 - 2*(qx^2 + qy^2)];
         
    % Calculate the X, Y, Z axes of the drone at this moment
    x_axis = origin + (R(:,1)' * scale);
    y_axis = origin + (R(:,2)' * scale);
    z_axis = origin + (R(:,3)' * scale);
    
    % Draw the drone's local frame (X=Red, Y=Green, Z=Blue)
    line([origin(1) x_axis(1)], [origin(2) x_axis(2)], [origin(3) x_axis(3)], 'Color', 'r', 'LineWidth', 1.5);
    line([origin(1) y_axis(1)], [origin(2) y_axis(2)], [origin(3) y_axis(3)], 'Color', 'g', 'LineWidth', 1.5);
    line([origin(1) z_axis(1)], [origin(2) z_axis(2)], [origin(3) z_axis(3)], 'Color', 'b', 'LineWidth', 1.5);
end

% 4. Formatting
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('Crazyflie 3D Flight Path with Quaternion Orientation');
view(3); % Set to 3D view