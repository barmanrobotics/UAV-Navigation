from pymavlink import mavutil
import time
import csv
import os
import socket
import threading

class Logger:
    def __init__(self, log_path, features):
        self.log_path = log_path
        self.features = features
        self.initialize()

    def initialize(self):
        if not os.path.isfile(self.log_path):
            with open(self.log_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(self.features)

    def write(self, features):
        with open(self.log_path, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(features)

os.makedirs("logs", exist_ok=True)

rx_log_path = "drone_rx_log.csv"
rx_log_path = os.path.join("logs", rx_log_path)
rx_logger = Logger(rx_log_path, ["timestamp", "latency"])

gps_log_path = "drone_gps_log.csv"
gps_log_path = os.path.join("logs", gps_log_path)
gps_logger = Logger(gps_log_path, ["timestamp", "lat", "lon", "alt"])

connection = mavutil.mavlink_connection("udpin:localhost:14550")
connection.wait_heartbeat()
print("Connected to Pixhawk.")

start_time = time.time()

def tx(conn):
    while True:
        cur_time = time.time()
        conn.sendall(str(cur_time).encode())
        time.sleep(1)

def rx(client):
    while True:
        msg = client.recv(1024).decode()
        rx_logger.write([time.time(), time.time()-float(msg)])

def update_gps():
    while True:
        msg = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        gps_logger.write([time.time(), msg.lat/1e7, msg.lon/1e7, msg.alt/1e3])

def start_server(host, port):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((host, port))
        tx_thread = threading.Thread(target=tx, args=(client,))
        tx_thread.daemon = True
        tx_thread.start()
        
        rx_thread = threading.Thread(target=rx, args=(client,))
        rx_thread.daemon = True
        rx_thread.start()

        gps_thread = threading.Thread(target=update_gps)
        gps_thread.daemon = True
        gps_thread.start()

        tx_thread.join()
        rx_thread.join()
        gps_thread.join()

        print("connected")

    except Exception as e:
        print(f"Connection error: {e}")
    finally:
        client.close()

start_server("localhost", 15000)