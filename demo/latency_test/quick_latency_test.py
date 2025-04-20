import csv
import time
from datetime import datetime
from ping3 import ping
from pymavlink import mavutil

HUB_IP = '10.203.121.89'
PORT = 14551

connection = mavutil.mavlink_connection(f'udpin:localhost:{PORT}')
connection.wait_heartbeat()

LOG_FILE = "drone_log.csv"

try:
    with open(LOG_FILE, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp_utc", "ping_ms", "latitude", "longitude", "altitude_m"])
except FileExistsError:
    pass

def get_gps_data():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1e3
        return lat, lon, alt
    return None, None, None

def get_ping():
    try:
        delay = ping("8.8.8.8", timeout=2)
        if delay is not None:
            return round(delay * 1000, 2)
    except Exception:
        pass
    return None

def log_data():
    timestamp = datetime.utcnow().isoformat()
    ping_ms = get_ping()
    lat, lon, alt = get_gps_data()

    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, ping_ms, lat, lon, alt])
        print(f"{timestamp} | Ping: {ping_ms}ms | Lat: {lat} | Lon: {lon} | Alt: {alt}m")

if __name__ == "__main__":
    print("Drone logger started")
    while True:
        log_data()
        time.sleep(5)