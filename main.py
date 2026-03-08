# Entry point of program
import queue
import threading
import time
from GSCore.drivers.motive_client import print_motive_data, start_motive_stream, start_print_thread
from GSCore.drivers.mock_motive_client import print_mock_data, start_mock_stream
from GSCore.gui.main_window import test_gui, start_gui
from common_classes import SystemState, Pose   
from GSCore.drivers.cf_client import connect_to_uav, test_cf_connection
from GSCore.core.commands import CommandQueue

def start_testbed():
    #test_gui()
    pose_queue = queue.Queue(maxsize=1)
    command_queue = CommandQueue()
    shared_state = SystemState()
    # Replace with real motive stream when wanted
    motive_client = start_mock_stream(pose_queue, shared_state)
    
    uav_thread = threading.Thread(
        target=connect_to_uav, 
        args=("radio://0/110/1M",), # Positional arguments go here (must be a tuple!)
        kwargs={                   # Keyword arguments go here
            "pose_queue": pose_queue, 
            "command_queue": command_queue, 
            "shared_state": shared_state
        },
        daemon=True # Daemon=True means this thread will die when you close the GUI
    )
    uav_thread.start()# Test Crazyflie connection with mock data stream
    start_gui(shared_state, command_queue=command_queue)
    
    

if __name__ == "__main__":
    start_testbed()