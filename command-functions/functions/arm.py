# Owen Bartlett
# Function to arm the motors

from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

def arm_drone():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    # Wait until armed
    while True:
        heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed")
            break
        time.sleep(0.5)
