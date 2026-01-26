# Not sure about this yet, might this file for explicit seperate thread to log and save data
import csv
import time

def logger_worker(state, stop_event, filename="flight_log.csv"):
    print(f"Logging to {filename}...")
    tic = time.time()
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        
        writer.writerow(['Timestamp', 'Motive_X', 'Motive_Y', 'Motive_Z'])
        
        while not stop_event.is_set():
            # 1. Get atomic snapshot of the entire system
            data = state.get_snapshot()
            
            # 2. Write to disk
            writer.writerow([
                data['t'], 
                data['mx'], data['my'], data['mz'], 
            ])
            
            # 3. Flush occasionally (optional, safer for crashes)
            # f.flush() 
            
           
            time.sleep(0.1)
            
    print("Logging complete.")


