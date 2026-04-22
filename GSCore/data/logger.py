import threading
import time
import csv
from datetime import datetime

#Class to handle logging as called from GUI
class FlightLogger:
    def __init__(self, swarm_dict):
        self.swarm_dict = swarm_dict
        self.is_logging = False
        self._log_thread = None

    def toggle_logging(self):
        """Called by the GUI button to flip the state."""
        if self.is_logging:
            self.stop()
        else:
            self.start()

    def start(self):
        if self.is_logging: return
        
        self.is_logging = True
        self._log_thread = threading.Thread(target=self._log_loop, daemon=True)
        self._log_thread.start()

    def stop(self):
        self.is_logging = False
        if self._log_thread:
            self._log_thread.join()

    def _log_loop(self):
        # Generate a unique filename: e.g., flight_log_20260308_153000.csv
        filename = f"flight_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        print(f"Started recording telemetry to {filename}")
        
        # Grab one snapshot just to extract the dictionary keys for the CSV headers
        initial_snap = self.swarm_dict[list(self.swarm_dict.keys())[0]].state.get_snapshot()
        initial_snap['agent_id'] = list(self.swarm_dict.keys())[0]
        fieldnames = list(initial_snap.keys())

        # Open the file and keep it open for the duration of the flight
        with open(filename, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            while self.is_logging:
                loop_start = time.time()
                
                # Grab fresh data from your lock-protected state
                for agent in self.swarm_dict.values():
                    snap = agent.state.get_snapshot()
                    snap['agent_id'] = agent.agent_id # Inject ID
                    snap['time'] = time.time() # Overwrite time with actual timestamp
                    writer.writerow(snap)
                
                # Force the OS to write to disk occasionally so data isn't lost in a crash
                csv_file.flush() 

                # Maintain 100Hz logging rate
                elapsed = time.time() - loop_start
                time.sleep(max(0.0, 0.01 - elapsed))
                
        print("Stopped recording. File saved successfully.")