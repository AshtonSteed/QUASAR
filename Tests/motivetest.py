import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from natnet import NatNetClient, DataFrame, RigidBody

# --- Motive (NatNet) Settings ---
MOTIVE_SERVER_IP = "127.0.0.1"  # IP of the machine running Motive
RIGID_BODY_ID = 1               # The ID of your "cf1" rigid body in Motive

# This global variable will hold the latest pose data
latest_pose = None

i =0

# This callback function is called every time Motive sends new data
def receive_motive_data(data_frame: DataFrame):
    global latest_pose, i
    # Find our rigid body in the data frame
    for body in data_frame.rigid_bodies:
        i +=1
        if body.id_num == RIGID_BODY_ID:
            # Extract position (x, y, z) and rotation (qx, qy, qz, qw)
            pos = body.pos
            rot = body.rot
            #markers = len(body.markers)
            err = body.marker_error # Mean marker error
            #com_err = err * (markers**-0.5 if markers > 0 else 1)
            
            # --- !!! IMPORTANT: Coordinate Tranwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww       sformation !!! ---
            # Make sure motive and CF coordinates align
                                             
            cf_x = pos[0]
            cf_y = pos[1] 
            cf_z = pos[2] 


            cf_qx =  rot[0]
            cf_qy =  -rot[1] 
            cf_qz =  -rot[2] 
            cf_qw =  rot[3]
            
            
            
            latest_pose = (cf_x, cf_y, cf_z, err)
            if i%15==0:
              
                print(f"Pos: X={cf_x:.3f} m, Y={cf_y:.3f} m, Z={cf_z:.3f} m MarkerError={err*1e3:.3f}mm  ", end="\r", flush=True)
            
            return

def main():
    # Initialize the Crazyflie drivers
   

    # --- Connect to Motive ---
    print("Connecting to Motive...")
    motive_client = NatNetClient(server_ip_address="127.0.0.1", local_ip_address="127.0.0.1", use_multicast=False)
    motive_client.connect()
    motive_client.on_data_frame_received_event.handlers.append(receive_motive_data)
    motive_client.run_async()
    print("Connected to Motive.")


if __name__ == "__main__":
    main()