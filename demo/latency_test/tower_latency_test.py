from pymavlink import mavutil
import time
import csv
import os
import threading
import socket

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

rx_log_path = "tower_rx_log.csv"
rx_log_path = os.path.join("logs", rx_log_path)
rx_logger = Logger(rx_log_path, ["timestamp", "msg_id"])

def tx(conn):
    while True:
        cur_time = time.time()
        conn.sendall(str(cur_time).encode())
        time.sleep(1)

def rx(client):
    while True:
        msg = client.recv(1024).decode()
        rx_logger.write([time.time(), time.time()-float(msg)])

def start_server(host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((host, port))
            s.listen()
            conn, addr = s.accept()
            print("Server connected.")
            
            tx_thread = threading.Thread(target=tx, args=(conn,))
            tx_thread.daemon = True
            tx_thread.start()
            
            rx_thread = threading.Thread(target=rx, args=(conn,))
            rx_thread.daemon = True
            rx_thread.start()

            tx_thread.join()
            rx_thread.join()
        except Exception as e:
            print(f"Server error on port {port}: {e}")

start_server("localhost", 15000)