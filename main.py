# Entry point of program
import queue
import queue
import threading
import time
from GSCore.drivers.motive_client import print_motive_data, start_motive_stream, start_print_thread
from GSCore.drivers.mock_motive_client import print_mock_data, start_mock_stream
from common_classes import SystemState, Pose
from GSCore.data.logger import logger_worker

def main():
    stop_event = threading.Event()
    pose_queue = queue.Queue(maxsize=1)
    shared_state = SystemState()
    motive_client = start_motive_stream(pose_queue, shared_state)

    # Start logging thread
    logger_thread = threading.Thread(target=logger_worker, args=(shared_state, stop_event, ))
    logger_thread.start()
    start_print_thread(pose_queue)
    time.sleep(30)  # Let it log for 10 seconds
    # Stop logging
    stop_event.set()
    logger_thread.join()

if __name__ == "__main__":
    main()