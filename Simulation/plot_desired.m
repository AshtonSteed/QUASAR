% plot_desired.m
% Generates 3D path based on Python UI variables

% 1. Create time vector
t_steps = linspace(0, ui_time, 1000); 
p_data = zeros(length(t_steps), 3);

% 2. Calculate points using your existing trajectory_gen function
for i = 1:length(t_steps)
    [p, ~, ~, ~] = trajectory_gen(t_steps(i), ui_shape_id, ui_width, ui_height, ui_speed, ui_time);
    p_data(i, :) = p';
end

% 3. Visualization
fig = figure(101); clf(fig);
set(fig, 'Name', 'Trajectory Preview', 'NumberTitle', 'off');
plot3(p_data(:,1), p_data(:,2), p_data(:,3), 'r-', 'LineWidth', 2);
hold on; grid on; axis equal;

% Markers
plot3(p_data(1,1), p_data(1,2), p_data(1,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % Start
plot3(p_data(end,1), p_data(end,2), p_data(end,3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % End

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title(['Desired Path - Shape ID: ', num2str(ui_shape_id)]);
view(3);