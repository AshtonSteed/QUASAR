# Entry point of program
import queue
import threading
import time

import cflib
from GSCore.core.agent import Agent
from GSCore.drivers.motive_client import print_motive_data, start_motive_stream, start_print_thread
from GSCore.drivers.mock_motive_client import print_mock_data, start_mock_stream
from GSCore.gui.main_window import start_gui

from GSCore.core.commands import CommandQueue

def start_testbed():
    

    # 1. Define Swarm Configuration
    swarm_config = [
        {"id": "CF_1", "uri": "radio://0/110/1M/E7E7E7E701", "motive_id": 1},
        {"id": "CF_2", "uri": "radio://0/110/1M/E7E7E7E702", "motive_id": 2},
        #{"id": "CF_3", "uri": "radio://0/110/1M/E7E7E7E703", "motive_id": 3},
    ]
    
    # 2. Instantiate Agents
    swarm_dict = {}
    for conf in swarm_config:
        agent = Agent(conf["id"], conf["uri"], conf["motive_id"])
        swarm_dict[conf["id"]] = agent
        
    # 3. Start Motive Router
    motive_client = start_motive_stream(swarm_dict)
    
    # 4. Connect Hardware
    cflib.crtp.init_drivers()
    for agent in swarm_dict.values():
        agent.connect_and_start()
            
    # 5. Launch GUI
    start_gui(swarm_dict)
    
    

if __name__ == "__main__":
    start_testbed()