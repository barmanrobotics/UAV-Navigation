import csv
import os
import cv2
import cv2.aruco as aruco
import socket
import struct
import numpy as np
import math
import time
from pymavlink import mavutil
import random

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")


def set_message_interval(rate_hz):
    """
    Sets the update rate for a specific MAVLink message using MAV_CMD_SET_MESSAGE_INTERVAL.

    :param connection: MAVLink connection object
    :param message_id: MAVLink message ID (e.g., 30 for ATTITUDE)
    :param rate_hz: Desired frequency in Hz
    """
    interval_us = int(1e6 / rate_hz)  # Convert Hz to microseconds
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,  # Confirmation
        30,
        interval_us,
        0, 0, 0, 0, 0
    )


def get_drone_attitude():
    """
    Connects to the drone via MAVLink and retrieves attitude (roll, pitch, yaw).
    
    :param connection_string: MAVLink connection string (e.g., 'udp:127.0.0.1:14550' or 'COM3' for serial)
    :return: Tuple (roll_deg, pitch_deg, yaw_deg) in degrees
    """
    # Wait for an ATTITUDE message
    msg = connection.recv_match(type='ATTITUDE', blocking=True)
    if msg:
        roll = msg.roll # Convert radians to degrees
        pitch = msg.pitch
        yaw = msg.yaw

        return roll, pitch, yaw
    return None

message_rate = 1
set_message_interval(message_rate)
prev_time = time.time()
while True:
    attitude = get_drone_attitude()
    cur_time = time.time()
    elapsed_time = cur_time-prev_time
    rate = 1/elapsed_time
    prev_time = cur_time
    print("Attiude is: ",attitude, " Rate is: ",rate, " Hz")