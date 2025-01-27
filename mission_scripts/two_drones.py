import threading
from pymavlink import mavutil
import time

# Add unique target system IDs for each drone
connections = [
    mavutil.mavlink_connection('udpin:localhost:14550'),  # Connection for Drone 1
    mavutil.mavlink_connection('udpin:localhost:14560')   # Connection for Drone 2
]

# Map connections to target system IDs
target_system_ids = [1, 2]  # IDs corresponding to SYSID_THISMAV for each drone

for idx, conn in enumerate(connections):
    print(f"Connected to Drone {idx + 1}, Target System: {conn.target_system}")

def set_mode(connection, target_system, mode):
    """
    Sets the flight mode of the drone.
    """
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} is not supported by drone {target_system}")
        return False
    connection.mav.set_mode_send(
        target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = connection.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Drone {target_system} mode changed to {mode}")
            break
        time.sleep(0.5)

def drone_mission(connection, target_system):
    """
    Executes the mission: takeoff, hover, and land.
    """
    set_mode(connection, target_system, "GUIDED")  # Set to GUIDED mode
    print(f"Arming drone {target_system}...")
    connection.mav.command_long_send(
        target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(2)  # Give time for arming

    print(f"Taking off drone {target_system} to 10 meters...")
    connection.mav.command_long_send(
        target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, 10
    )
    time.sleep(20)  # Hover for 20 seconds

    print(f"Landing drone {target_system}...")
    connection.mav.command_long_send(
        target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )



for connection in connections:
    connection.wait_heartbeat()
    print(f"Connected to drone {connection.target_system}")

# Create threads for each drone mission
threads = []
for idx, connection in enumerate(connections):

    thread = threading.Thread(target=drone_mission, args=(connection, target_system_ids[idx]))
    threads.append(thread)
    thread.start()

# Wait for all threads to finish
for thread in threads:
    thread.join()

print("Mission completed for all drones.")