import time
import threading
import math
import queue

# Import your shared classes
# (Adjust import based on your file structure)
from GSCore.drivers.motive_client import print_worker
from common_classes import SystemState, Pose

def mock_motive_worker(swarm_dict):
    """Generates fake drone position data at 120Hz"""
    print("[MOCK] Started. Generates a circle pattern.")
    
    start_time = time.time()
    
    while True:
        t = time.time() - start_time
        
        for i, agent in enumerate(swarm_dict.values()):
            system_state = agent.state
            pose_queue = agent.pose_queue
            
            # 1. Create Fake Data 
            # Phase shift based on index (i) so the drones are spaced out in a circle
            phase_offset = 2*math.pi * (i / len(swarm_dict))
            
            # Radius 1.0m, Height 1.0m, Period 5s
            x = 1.0 * math.cos(t * (2 * math.pi / 5.0) + phase_offset)
            y = 1.0 * math.sin(t * (2 * math.pi / 5.0) + phase_offset)
            z = 1.0 + 0.1 * math.sin(t * 2 + phase_offset) # Slight hover bob
            
            # Quaternions (Just flat for now)
            qx, qy, qz, qw = 0.0, 0.0, math.sin(i * t), 1.0

            pose = Pose.from_motive([x, y, z], [qx, qy, qz, qw], 0.0, True)
            
            # 2. Update STATE (For Logger/GUI) Safely using the thread lock
            with system_state.lock:
                system_state.motive_pose = pose
                
                # Mock the estimator as well, otherwise the GUI "Estimate" graphs remain blank
                system_state.estimate_pose.x = x
                system_state.estimate_pose.y = y
                system_state.estimate_pose.z = z
                
                system_state.estimate_pose.qx = qx
                system_state.estimate_pose.qy = qy
                system_state.estimate_pose.qz = qz
                system_state.estimate_pose.qw = qw
                
                # Mock the health stats so the GUI flags light up
                system_state.battery_voltage = 3.9
                system_state.armed = True
                system_state.flying = True
                
                # CRITICAL: Update the main time tracker so the GUI knows to draw a new frame!
                system_state.time = time.time() 
                
            # 3. Manage the Queue
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

def start_mock_stream(swarm_dict):
    
    t = threading.Thread(
        target=mock_motive_worker, 
        args=(swarm_dict,), 
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
    