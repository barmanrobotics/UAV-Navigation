# Owen Bartlett
# This is a script designed to mimic a complete flight.
# It should take off, fly to a predetermined position, stabililze, lower in altitude,
# then return back to the original gps location. 

from pymavlink import mavutil # type: ignore
import time
import math

target_alt = 2

# Start a connection listening to a UDP port or the Serial port on the Raspi
connection = mavutil.mavlink_connection('dev/ttyAMA0', baudrate = 921600) #'udpin:localhost:14551' >> SITL #'dev/ttyAMA0', baudrate = 921600 >> RasPI

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

def set_mode(mode):
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} is not supported")
        return False
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = connection.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            break
        time.sleep(0.5)

# Arm the drone
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

# Take off to a specified altitude
def takeoff(altitude):
    print(f"Taking off to {altitude} meters...")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_alt:.2f} meters")
        if current_alt >= altitude * 0.95:  # Reached 95% of target altitude
            print("Reached target altitude")
            break
        time.sleep(0.5)


# Execute the mission
set_mode("GUIDED")
arm_drone()
takeoff(target_alt)
print("waiting 10 seconds before landing")
time.sleep(10)
print("Time to land")
set_mode("LAND")
print("landing")