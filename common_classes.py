import time
import threading
from cflib.utils.encoding import decompress_quaternion
import scipy

class Pose:
    def __init__(self, x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0, error=0.0, valid=False):
        self.x = x
        self.y = y
        self.z = z
        self.qx = qx
        self.qy = qy
        self.qz = qz
        self.qw = qw
        self.error = error
        self.valid = valid

    @classmethod
    def from_motive(cls, pos, rot, err, valid):
       
        
        return cls(
            x = pos[0],
            y = pos[1],      
            z = pos[2],      
            qx = rot[0],
            qy = rot[2],     # Swapped/Negated based on axis rule
            qz = rot[1],
            qw = rot[3],
            error = err,
            valid = valid
        )
    
    def xyz(self):
        return [self.x, self.y, self.z]
    
    

## Thread-Safe class storing all logged information. 
class SystemState:
    def __init__(self):
        self.lock = threading.Lock()
        
        # 1. KINEMATICS (Ground Truth)
        self.motive_pose = Pose()
        self.time = 0.0
        
        # 2. KINEMATICS (Onboard Estimate & Setpoints)
        self.estimate_pose = Pose() # What the drone thinks its pose is
        self.target_setpoint = (0.0, 0.0, 0.0) # Commanded X, Y, Z (Temp for now)
        
        # 3. DYNAMICS & IMU
        self.angular_rates = (0.0, 0.0, 0.0) # p, q, r (Gyro/estimator)
        self.linear_velocity = (0.0, 0.0, 0.0) # vx, vy, vz (estimator)
        self.linear_accel = (0.0, 0.0, 0.0)  # ax, ay, az (accelerometer)
        
        # 4. CONTROL AUTHORITY
        self.motor_commands = (0.0, 0.0, 0.0, 0.0) # m1, m2, m3, m4
        
        # 5. SYSTEM HEALTH & TIMING
        self.battery_voltage = 0.0
        self.armed = False
        self.flying = False
        self.tumbled = False
        self.locked = False
        self.crashed = False
        
    def update_motive(self, pose):
        """Called by the Motive/OptiTrack listener thread"""
        with self.lock:
            self.motive_pose = pose
            self.time = time.time()

    # Callback/update for pose logging
    def pose_data_callback(self, timestamp, data, logconf):
        x = data['stateEstimateZ.x'] / 1000.0 # Convert mm to m
        y = data['stateEstimateZ.y'] / 1000.0
        z = data['stateEstimateZ.z'] / 1000.0
        compressed_quat = data['stateEstimateZ.quat']
       #Decompress the quaternion
        qx, qy, qz, qw = decompress_quaternion(compressed_quat)

        #Quickly lock, update the shared state, and release
        with self.lock:
            self.time = timestamp
            self.estimate_pose.x = x
            self.estimate_pose.y = y
            self.estimate_pose.z = z
            self.estimate_pose.qx = qx
            self.estimate_pose.qy = qy
            self.estimate_pose.qz = qz
            self.estimate_pose.qw = qw
    
    def dyn_data_callback(self, timestamp, data, logconf):
        l_v = (
                data['stateEstimateZ.vx'] / 1000.0, # mm/s to m/s
                data['stateEstimateZ.vy'] / 1000.0,
                data['stateEstimateZ.vz'] / 1000.0
            )
        l_a = (
                data['stateEstimateZ.ax'] / 1000.0, # mm/s^2 to m/s^2
                data['stateEstimateZ.ay'] / 1000.0,
                data['stateEstimateZ.az'] / 1000.0
            )
        
        pqr = (
                data['stateEstimateZ.rateRoll'] / 1000.0, # milirads/s to rad/s
                data['stateEstimateZ.ratePitch'] / 1000.0,
                data['stateEstimateZ.rateYaw'] / 1000.0
            )
        with self.lock:
            self.time = timestamp
            self.linear_velocity = l_v
            self.linear_accel = l_a
            self.angular_rates = pqr
   
    def motor_data_callback(self, timestamp, data, logconf):
        motors = (
            data['motor.m1'] / 65535.0, # Normalize to 0-1
            data['motor.m2'] / 65535.0,
            data['motor.m3'] / 65535.0,
            data['motor.m4'] / 65535.0
        )
        with self.lock:
            self.time = timestamp
            self.motor_commands = motors
            
    def health_data_callback(self, timestamp, data, logconf):
        vbat = data['pm.vbatMV'] / 1000.0 #Battery Voltage in V
        # Decode the supervisor state (From CF docs about supervisor and uav states)
        #https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/supervisor.c
        supervisor_info = data['supervisor.info']
        is_armed = (supervisor_info & 0b0000000000000010) > 0
        is_flying = (supervisor_info & 0b0000000000010000) > 0
        is_tumbled = (supervisor_info & 0b0000000000100000) > 0
        is_locked = (supervisor_info & 0b0000000001000000) > 0
        is_crashed = (supervisor_info & 0b0000000010000000) > 0
        
        with self.lock:
            self.time = timestamp
            self.battery_voltage = vbat
            self.armed = is_armed
            self.flying = is_flying
            self.tumbled = is_tumbled
            self.locked = is_locked
            self.crashed = is_crashed
        
    
            
    def get_snapshot(self):
        """
        Returns a flat, fast copy of the current state for the GUI to log and plot.
        Grabbing individual properties prevents reference leaks.
        """ 
        with self.lock:
            return {
                # Timing
                't': self.time,
                
                # Ground Truth (Motive)
                'mx': self.motive_pose.x,
                'my': self.motive_pose.y,
                'mz': self.motive_pose.z,
                'mqx': self.motive_pose.qx,
                'mqy': self.motive_pose.qy,
                'mqz': self.motive_pose.qz,
                'mqw': self.motive_pose.qw,
                'm_error': self.motive_pose.error,
                
                # State Estimate (Onboard)
                'ex': self.estimate_pose.x,
                'ey': self.estimate_pose.y,
                'ez': self.estimate_pose.z,
                'eqx': self.estimate_pose.qx,
                'eqy': self.estimate_pose.qy,
                'eqz': self.estimate_pose.qz,
                'eqw': self.estimate_pose.qw,
                'vx': self.linear_velocity[0],
                'vy': self.linear_velocity[1],
                'vz': self.linear_velocity[2],
                'ax': self.linear_accel[0],
                'ay': self.linear_accel[1],
                'az': self.linear_accel[2],
                
                # Setpoint
                'sx': self.target_setpoint[0],
                'sy': self.target_setpoint[1],
                'sz': self.target_setpoint[2],
                
                # Dynamics
                'p': self.angular_rates[0],
                'q': self.angular_rates[1],
                'r': self.angular_rates[2],
                
                # Actuators
                'm1': self.motor_commands[0],
                'm2': self.motor_commands[1],
                'm3': self.motor_commands[2],
                'm4': self.motor_commands[3],
                
                # Health
                'vbat': self.battery_voltage,
                'armed': self.armed,
                'flying': self.flying,
                'tumbled': self.tumbled,
                'locked': self.locked,
                'crashed': self.crashed
            }
        