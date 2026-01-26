import time
import threading

class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, error=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.error = error

    @classmethod
    def from_motive(cls, pos, rot, err):
       
        
        return cls(
            x = pos[0],
            y = pos[1],      
            z = pos[2],      
            qx = rot[0],
            qy = rot[2],     # Swapped/Negated based on axis rule
            qz = rot[1],
            qw = rot[3],
            error = err
        )
    
    def xyz(self):
        return [self.x, self.y, self.z]
    
    

# Thread-Safe class storing all logged information. Slow, but accurate.
class SystemState:
    def __init__(self):
        self.lock = threading.Lock()
        # Motive Data
        self.pose = Pose()
        self.motive_time = 0.0
        
        # Placeholder Drone Data (From Radio)
        self.battery_voltage = 0.0
        self.target_setpoint = (0, 0, 0)
        
    def update_motive(self, pose):
        with self.lock:
            self.pose = pose
            self.motive_time = time.time()
            
    def update_drone(self, vbat, setpoint):
        with self.lock:
            self.battery_voltage = vbat
            self.target_setpoint = setpoint
            
    def get_snapshot(self):
        """Returns a copy of the current state for logging"""
        with self.lock:
            return {
                't': self.motive_time,
                'mx': self.pose.x,
                'my': self.pose.y,
                'mz': self.pose.z,
            }
        