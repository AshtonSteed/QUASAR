% test_flight.m
figure(1); clf; hold on; grid on; axis equal; view(3);

% Extract path and orientation
p = p_out.Data; 
q = q_out.Data;

% Plot the blue flight path
plot3(p(:,1), p(:,2), p(:,3), 'b', 'LineWidth', 2);

% Plot a single red "Drone" marker at the start and green at the end
plot3(p(1,1), p(1,2), p(1,3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
plot3(p(end,1), p(end,2), p(end,3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Draw the final orientation triad
origin = p(end,:);
cur_q = q(end,:);
qw = cur_q(1); qx = cur_q(2); qy = cur_q(3); qz = cur_q(4);
R = [1 - 2*(qy^2 + qz^2),   2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy);
     2*(qx*qy + qw*qz),     1 - 2*(qx^2 + qz^2), 2*(qy*qz - qw*qx);
     2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),   1 - 2*(qx^2 + qy^2)];

% X-axis (Red), Y-axis (Green), Z-axis (Blue)
quiver3(origin(1), origin(2), origin(3), R(1,1), R(2,1), R(3,1), 0.5, 'r', 'LineWidth', 2);
quiver3(origin(1), origin(2), origin(3), R(1,2), R(2,2), R(3,2), 0.5, 'g', 'LineWidth', 2);
quiver3(origin(1), origin(2), origin(3), R(1,3), R(2,3), R(3,3), 0.5, 'b', 'LineWidth', 2);

title('Simple Flight Test: Red=Start, Green=End');
xlabel('X'); ylabel('Y'); zlabel('Z');