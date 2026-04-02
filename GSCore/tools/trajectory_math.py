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
    def helical(start_pos, center, radius, sweep_angle_rad, target_z, n_waypoints):
        """
        Generates a circular or helical path. 
        For a flat circle, set target_z equal to start_pos Z.
        """
        sx, sy, sz, _ = start_pos
        cx, cy = center
        waypoints = []

        # Calculate the starting angle based on current position relative to center
        start_angle = math.atan2(sy - cy, sx - cx)

        for k in range(n_waypoints):
            u = k / (n_waypoints - 1) if n_waypoints > 1 else 1.0
            
            # Current angle along the arc
            theta = start_angle + (u * sweep_angle_rad)
            
            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)
            z = sz + u * (target_z - sz)
            
            # Keep the nose pointed tangent to the arc (direction of travel)
            # Add pi/2 (90 degrees) to the positional angle
            yaw = theta + (math.pi / 2)
            
            waypoints.append((x, y, z, yaw))
            
        return waypoints