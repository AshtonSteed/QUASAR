import queue
from common_classes import SystemState
from GSCore.core.commands import CommandQueue
from GSCore.drivers.cf_client import CrazyflieDriver

class Agent:
    """
    Represents a single UAV node within the swarm.
    Encapsulates its state, command queues, and hardware driver.
    """
    def __init__(self, agent_id: str, uri: str, motive_id: int):
        self.agent_id = agent_id
        self.uri = uri
        self.motive_id = motive_id
        
        # Thread-safe state and communication queues
        self.state = SystemState()
        self.pose_queue = queue.Queue(maxsize=1)
        self.command_queue = CommandQueue()
        
        # The hardware driver specific to this agent
        self.driver = CrazyflieDriver(self.uri, self.pose_queue, self.command_queue, shared_state=self.state)
        
    def connect_and_start(self):
        """Helper to spin up this specific agent's driver."""
        if self.driver.connect():
            
            
            print(f"Agent {self.agent_id} connected successfully.")
            self.driver.start()
            
            # TODO: TEMPORARY HACK - test CF1 using mellinger vs cf2 using PID
            #if self.agent_id == "CF_2":
            #    self.driver.cf.param.set_value('stabilizer.controller', '2') # Use Mellinger Controller
            #    self.driver.cf.param.set_value('ctrlMel.mass', '32.0e-3') # Set the mass of the drone, ~36g?
            return True
        
        print(f"Agent {self.agent_id} failed to connect.")
        return False