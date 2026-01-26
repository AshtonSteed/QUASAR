# File responsible for handling communication with OptiTrack Motive software
# Connect, translate, and store 6DOF position
# Creates connection to NatNet, starts a 'listener' thread for incoming 6DOF states



import math
from natnet import NatNetClient, DataFrame
import queue
from common_classes import SystemState, Pose


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
            
            x, y, z, err = packet.x, packet.y, packet.z, packet.error
            
            # \r overwrites the same line so your terminal doesn't flood
            print(f"Motive: X={x:.3f} Y={y:.3f} Z={z:.3f}, ERR={1000 * err:.3f} mm  ", end="\r")
            
        except queue.Empty:
            pass

# Create a motive listener thread to receive data
def start_motive_stream(pose_queue: queue.Queue, shared_state: SystemState = None):
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
                
                # Package 6DOF state from Motive
                latest_pose = Pose.from_motive(pos, rot, err)
                # Update shared logger state
                if shared_state:
                    shared_state.pose = latest_pose
                # Push to the queue
                try:
                    pose_queue.put_nowait(latest_pose)
                except queue.Full:
                    pose_queue.get_nowait()
                    pose_queue.put_nowait(latest_pose)
            
                return

    motive_client = NatNetClient(server_ip_address=MOTIVE_SERVER_IP, local_ip_address="127.0.0.1", use_multicast=False)
    motive_client.connect()
    motive_client.on_data_frame_received_event.handlers.append(receive_motive_data)
    motive_client.run_async()
    
    print("Connectd to Motive.")
    return motive_client
    
    
def print_motive_data():
    #Example useage in main thread
    # Main thread creates pose queue, motive client thread, and any other threads
    pose_queue = queue.Queue(maxsize=1)
    shared_state = SystemState()
    motive_client = start_motive_stream(pose_queue, shared_state)
    p_thread = threading.Thread(target=print_worker, args=(pose_queue,))
    p_thread.start()
    #report_threads()
    pass
    


# Code to debug number of threads
import threading

def report_threads():
    print(f"Total Active Threads: {threading.active_count()}")
    for thread in threading.enumerate():
        print(f" - {thread.name} (ID: {thread.ident}, Daemon: {thread.daemon})")


