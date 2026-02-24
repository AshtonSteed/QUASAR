# Entry point of program
import queue
import threading
import time
from GSCore.drivers.motive_client import print_motive_data, start_motive_stream, start_print_thread
from GSCore.drivers.mock_motive_client import print_mock_data, start_mock_stream
from GSCore.gui.main_window import test_gui
from common_classes import SystemState, Pose
from GSCore.data.logger import logger_worker
from GSCore.drivers.cf_client import test_cf_connection

def main():
    test_gui()
    #test_cf_connection("radio://0/80/2M") # Test Crazyflie connection with mock data stream
    

if __name__ == "__main__":
    main()