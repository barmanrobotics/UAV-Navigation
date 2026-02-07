from pymavlink import mavutil
import time, math

port = 14551

print(f"Connecting to MAVLink on {port}...")
connection = mavutil.mavlink_connection(f'udpin:127.0.0.1:{port}')
print(f"Waiting for heartbeat on {port}...")
connection.wait_heartbeat()
print(f"Heartbeat received from system {connection.target_system}, component {connection.target_component}")

def get_gps_info():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    if msg:
        lat, lon, alt, rel_altHopk, hdg = msg.lat/1e7, msg.lon/1e7, msg.alt/1000.0, msg.relative_alt/1000.0, msg.hdg

def continuous_readout():
    while True:
        get_gps_info()
        time.sleep(.1)