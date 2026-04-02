% setup_and_run_sim.m

% 1. Flight Parameters (UI)
ui_shape_id = 1;      % 1: Helix, 2: Fig8, 3: Square
ui_width = 2.0;       % Radius or width (m)
ui_height = 1.5;      % Hover altitude (m)
ui_speed = 1.0;       % Speed modifier
ui_time = 20.0;       % Total flight time (s)

% 2. Crazyflie Physics Parameters (SI Units)
m = 0.027;            % Mass of Crazyflie (kg)
g = 9.81;             % Gravity (m/s^2)
J = [1.66e-5, 0, 0; 
     0, 1.66e-5, 0; 
     0, 0, 2.93e-5];  

% 3. Load the model
model_name = 'crazyflie_sim';
load_system(model_name); 

% 4. Sync simulation time and Fix the "Clock Clash"
set_param(model_name, 'StopTime', num2str(ui_time));
set_param(model_name, 'FixedStep', '1/600'); 

% 5. Run the Simulation
disp('Workspace loaded. Starting simulation...');
simOut = sim(model_name);
disp('Simulation complete!');

% ==========================================
% 6. GENERATE 3D FLIGHT PLOT
% ==========================================
disp('Generating 3D Flight Path...');

% --- A. GENERATE DESIRED PATH (BLUE LINE) ---
% Extract the exact time array used by the simulation
if isprop(simOut, 'tout')
    t_sim = simOut.tout;
else
    t_sim = linspace(0, ui_time, 1000)'; % Fallback if tout isn't saved
end

% Pre-allocate arrays for speed
xd = zeros(length(t_sim), 1);
yd = zeros(length(t_sim), 1);
zd = zeros(length(t_sim), 1);

% Call the trajectory generator for every time step
for i = 1:length(t_sim)
    [p_target, ~, ~, ~] = trajectory_gen(t_sim(i), ui_shape_id, ui_width, ui_height, ui_speed, ui_time);
    xd(i) = p_target(1);
    yd(i) = p_target(2);
    zd(i) = p_target(3);
end

% --- B. EXTRACT ACTUAL PATH (RED LINE) ---
if isprop(simOut, 'p_out') 
    if isa(simOut.p_out, 'timeseries')
        pos_data = simOut.p_out.Data;
    else
        pos_data = simOut.p_out; 
    end
    
    if size(pos_data, 2) == 3 && size(pos_data, 1) > 3
        x = pos_data(:, 1); y = pos_data(:, 2); z = pos_data(:, 3);
    else
        x = pos_data(1, :); y = pos_data(2, :); z = pos_data(3, :);
    end
    
    % --- C. PLOT EVERYTHING ---
    figure('Name', 'QUASAR Flight Trajectory', 'Color', 'w', 'Position', [100, 100, 800, 600]);
    hold on; grid on;
    
    % Plot Desired Path (Blue Dashed)
    plot3(xd, yd, zd, 'b--', 'LineWidth', 2, 'DisplayName', 'Desired Path');
    
    % Plot Actual Path (Red Solid)
    plot3(x, y, z, 'r-', 'LineWidth', 2, 'DisplayName', 'Actual Path');
    
    % Mark Start and End points
    plot3(x(1), y(1), z(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot3(x(end), y(end), z(end), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'End');
    
    view(3); 
    xlabel('X Position (m)', 'FontWeight', 'bold');
    ylabel('Y Position (m)', 'FontWeight', 'bold');
    zlabel('Z Position (Altitude) (m)', 'FontWeight', 'bold');
    title(sprintf('Crazyflie Trajectory - Shape ID: %d', ui_shape_id), 'FontSize', 14);
    legend('Location', 'best', 'FontSize', 12);
    axis equal; 
    
    disp('Plot generated successfully!');
else
    disp('ERROR: Could not find "p_out" in simOut.');
end