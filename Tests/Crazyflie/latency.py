import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from threading import Event

# URI to the Crazyflie to connect to
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Event object to handle the asynchronous callback
got_param_event = Event()

def param_update_callback(name, value):
    """Callback for when the parameter is updated"""
    # Unblock the main thread
    got_param_event.set()

def measure_latency(scf):
    cf = scf.cf
    
    # We will read 'pm.vbat' (Battery voltage). It's a safe, standard parameter.
    param_name = 'pm.lowVoltage'
    
    # Attach the callback to this specific parameter
    cf.param.add_update_callback(group='pm', name='lowVoltage', cb=param_update_callback)
    
    print(f"Measuring Application RTT using {param_name}...")
    print("-" * 30)

    latencies = []

    for i in range(150): # Run 20 tests
        # 1. Clear the event flag
        got_param_event.clear()
        
        # 2. Start Timer
        start_time = time.perf_counter()
        
        # 3. Request the update (Sends packet to drone)
        cf.param.request_param_update(param_name)
        
        # 4. Wait for the reply (Blocks until callback fires)
        # Timeout after 1 second to prevent hanging on lost packets
        success = got_param_event.wait(timeout=1.0)
        
        # 5. Stop Timer
        end_time = time.perf_counter()
        
        if success:
            rtt_ms = (end_time - start_time) * 1000
            latencies.append(rtt_ms)
            print(f"Ping {i+1}: {rtt_ms:.2f} ms")
        else:
            print(f"Ping {i+1}: Timed out (Packet Loss)")
        
        # Sleep slightly to let the radio queue clear
        time.sleep(0.05)

    if latencies:
        avg = sum(latencies[2:]) / len(latencies[2:])
        print("-" * 30)
        print(f"Average Round-Trip Time: {avg:.2f} ms")
        print(f"Min: {min(latencies):.2f} ms | Max: {max(latencies[2:]):.2f} ms | Startup: {latencies[1]:.2f} ms")

if __name__ == '__main__':
    cflib.crtp.init_drivers()
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        measure_latency(scf)