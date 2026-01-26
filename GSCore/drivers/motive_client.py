# File responsible for handling communication with OptiTrack Motive software
# Connect, translate, and store 6DOF position
# Creates connection to NatNet, starts a 'listener' thread for incoming 6DOF states



import math
from natnet import NatNetClient, DataFrame
import queue

# --- Motive (NatNet) Settings ---
MOTIVE_SERVER_IP = "127.0.0.1"  # LocalHost IP
RIGID_BODY_ID = 1             #   The ID the UAV in Motive  
N = 3  # Number of markers on the rigid body


# Worker thread function for printing pose data
# Uses a seperate thread from the listener to avoid any slowdowns or blocking
def print_worker(pose_queue):
    while True:
        try:
            # We use only the LATEST data.
            # If the printer is slow, it skips frames.
            packet = pose_queue.get(timeout=1.0)
            
            x, y, z = packet[0], packet[1], packet[2]
            
            # \r overwrites the same line so your terminal doesn't flood
            print(f"Motive: X={x:.3f} Y={y:.3f} Z={z:.3f}, ERR={1000 * packet[7]:.3f} mm  ", end="\r")
            
        except queue.Empty:
            pass

# Create a motive listener thread to receive data
def start_motive_stream(pose_queue: queue.Queue):
    # This callback function is called every time Motive sends new data
   
    print("Connecting to Motive...")
    

    # This callback function is called every time Motive sends new data
    def receive_motive_data(data_frame: DataFrame):
        # Find our rigid body in the data frame
        for body in data_frame.rigid_bodies:
            if body.id_num == RIGID_BODY_ID:
                # Extract position (x, y, z) and rotation (qx, qy, qz, qw)
                pos = body.pos
                rot = body.rot
                err = body.marker_error  / math.sqrt(N)# Mean marker error

                # --- !!! IMPORTANT: Coordinate transformation !!! ---
                # Make sure motive and CF coordinates align
                                                
                cf_x = pos[0]
                cf_y = pos[1] 
                cf_z = pos[2] 


                cf_qx =  rot[0]
                cf_qy =  -rot[1] 
                cf_qz =  -rot[2] 
                cf_qw =  rot[3]
                
                
                latest_pose = (cf_x, cf_y, cf_z, cf_qx, cf_qy, cf_qz, cf_qw, err)
                # Push to the passed queue
                try:
                    pose_queue.put_nowait(latest_pose)
                except queue.Full:
                    try:
                        pose_queue.get_nowait()
                        pose_queue.put_nowait(latest_pose)
                    except queue.Empty:
                        pass
            
                return

    motive_client = NatNetClient(server_ip_address=MOTIVE_SERVER_IP, local_ip_address="127.0.0.1", use_multicast=False)
    motive_client.connect()
    motive_client.on_data_frame_received_event.handlers.append(receive_motive_data)
    motive_client.run_async()
    
    print("Connectd to Motive.")
    return motive_client
    
    
def main():
    #Example useage in main thread
    # Main thread creates pose queue, motive client thread, and any other threads
    pose_queue = queue.Queue()
    motive_client = start_motive_stream(pose_queue)
    p_thread = threading.Thread(target=print_worker, args=(pose_queue,))
    p_thread.start()
    #report_threads()
    pass
    

import threading

def report_threads():
    print(f"Total Active Threads: {threading.active_count()}")
    for thread in threading.enumerate():
        print(f" - {thread.name} (ID: {thread.ident}, Daemon: {thread.daemon})")




if __name__ == "__main__":
    main()
    