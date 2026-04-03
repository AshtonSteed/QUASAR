# trajectory_math.py
import math

class TrajectoryGenerator:
    """
    Generates discrete waypoints for continuous 3D arcs.
    All methods return a list of tuples: [(x, y, z, yaw), ...]
    """

    @staticmethod
    def linear(start_pos, target_pos, n_waypoints):
        sx, sy, sz, syaw = start_pos
        tx, ty, tz, tyaw = target_pos
        waypoints = []

        for k in range(n_waypoints):
            # u normalizes the step from 0.0 to 1.0
            u = k / (n_waypoints - 1) if n_waypoints > 1 else 1.0
            
            x = sx + u * (tx - sx)
            y = sy + u * (ty - sy)
            z = sz + u * (tz - sz)
            # Linear interpolation for yaw
            yaw = syaw + u * (tyaw - syaw)
            
            waypoints.append((x, y, z, yaw))
            
        return waypoints

    @staticmethod
    def helical(start_pos, center, target_radius, sweep_angle_rad, target_z, n_waypoints):
        """
        Generates a helical path. Locks the radius to the actual distance between 
        the starting position and the center to guarantee a perfect cylinder with no jogging.
        """
        sx, sy, sz, _ = start_pos
        cx, cy = center
        waypoints = []

        # 1. Lock the radius to the physical distance to the center
        dx = sx - cx
        dy = sy - cy
        true_radius = math.sqrt(dx**2 + dy**2)

        # Failsafe: If the drone is sitting exactly on the center coordinate, 
        # it uses the UI's target_radius to push out and start the circle.
        if true_radius < 0.001:
            true_radius = target_radius 

        start_angle = math.atan2(dy, dx)

        for k in range(n_waypoints):
            u = k / (n_waypoints - 1) if n_waypoints > 1 else 1.0
            
            theta = start_angle + (u * sweep_angle_rad)
            
            # Use the locked true_radius, eliminating the spiral effect
            x = cx + true_radius * math.cos(theta)
            y = cy + true_radius * math.sin(theta)
            z = sz + u * (target_z - sz)
            
            # Keep nose pointed tangent to the curve
            yaw = theta + (math.pi / 2)
            
            waypoints.append((x, y, z, yaw))
            
        return waypoints