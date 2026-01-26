# Not sure about this yet, might this file for explicit seperate thread to log and save data
import csv

def logger_worker(filename="flight_log.csv"):
    print(f"Logging to {filename}...")
    
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        # Header
        writer.writerow(['Timestamp', 'Motive_X', 'Motive_Y', 'Motive_Z', 'Bat_V', 'Set_X', 'Set_Y', 'Set_Z'])
        
        while True:
            # 1. Get atomic snapshot of the entire system
            data = state.get_snapshot()
            
            # 2. Write to disk
            writer.writerow([
                data['t'], 
                data['mx'], data['my'], data['mz'], 
                data['vbat'],
                data['sx'], data['sy'], data['sz']
            ])
            
            # 3. Flush occasionally (optional, safer for crashes)
            # f.flush() 
            
            # 4. Rate Limit (e.g., 50Hz)
            # No need to log at 120Hz for post-analysis usually
            time.sleep(0.02)