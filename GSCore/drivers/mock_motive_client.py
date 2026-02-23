import time
import threading
import math
import queue

# Import your shared classes
# (Adjust import based on your file structure)
from GSCore.drivers.motive_client import print_worker
from common_classes import SystemState, Pose

def mock_motive_worker(pose_queue, system_state):
    """Generates fake drone position data at 120Hz"""
    print("[MOCK] Started. Generates a circle pattern.")
    
    start_time = time.time()
    
    while True:
        # 1. Create Fake Data (e.g., a Circle)
        t = time.time() - start_time
        
        # Radius 1.0m, Height 1.0m, Period 5s
        x = 1.0 * math.cos(t * (2 * math.pi / 5.0))
        y = 1.0 * math.sin(t * (2 * math.pi / 5.0))
        z = 1.0 + 0.1 * math.sin(t * 2) # Slight hover bob
        #x, y, z = 0, 0, 0
        
        # Quaternions (Just flat for now)
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0



        pose = Pose.from_motive([x, y, z], [qx, qy, qz, qw], 0.0, True)
        # 2. Update STATE (For Logger/GUI)
        if system_state:
            system_state.pose = pose
            system_state.motive_time = time.time()
            
       
        
        try:
            pose_queue.put_nowait(pose)
        except queue.Full:
            try:
                pose_queue.get_nowait() # Trash old
                pose_queue.put_nowait(pose) # Push new
            except queue.Empty:
                pass
                
        # 4. Sleep to simulate 120Hz
        time.sleep(1/120.0)

def start_mock_stream(pose_queue, system_state=None):
    
    t = threading.Thread(
        target=mock_motive_worker, 
        args=(pose_queue, system_state), 
        daemon=True,
        name="MockMotive"
    )
    t.start()
    return t


def print_mock_data():
    #Example useage in main thread
    # Main thread creates pose queue, motive client thread, and any other threads
    pose_queue = queue.Queue(maxsize=1)
    shared_state = SystemState()
    motive_client = start_mock_stream(pose_queue, shared_state)
    p_thread = threading.Thread(target=print_worker, args=(pose_queue,))
    p_thread.start()
    #report_threads()
    pass
    