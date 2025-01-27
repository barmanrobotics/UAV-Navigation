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

# Shared data for communication
shared_data = {
    "waypoint": None,  # Stores the waypoint for Drone 2
    "signal": None,    # Stores signals between drones
    "lock": threading.Lock()  # Synchronization lock
}

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

def move_to_waypoint(connection, target_system, x_offset, y_offset, z_offset):
    """
    Moves the drone to a waypoint relative to its current position.
    """
    print(f"Drone {target_system} moving to waypoint: x={x_offset}, y={y_offset}, z={z_offset}...")
    connection.mav.set_position_target_local_ned_send(
        0,
        target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        int(0b110111111000),  # Position control bits
        x_offset, y_offset, z_offset,
        0, 0, 0,  # Velocity
        0, 0, 0,  # Acceleration
        0, 0  # Yaw, Yaw rate
    )
    time.sleep(5)

def send_signal(signal_code):
    """Sets a signal in shared data for communication."""
    with shared_data["lock"]:
        shared_data["signal"] = signal_code

def wait_for_signal(expected_signal):
    """Waits for a specific signal from shared data."""
    while True:
        with shared_data["lock"]:
            if shared_data["signal"] == expected_signal:
                print(f"Received signal: {expected_signal}")
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
    time.sleep(10)  # Hover for 20 seconds

    if target_system == 1:
        # Drone 1 performs its waypoint mission and sends the waypoint to Drone 2
        waypoint = (10, 0, -10)
        print(f"Drone {target_system} moving to its waypoint: {waypoint}")
        move_to_waypoint(connection, target_system, *waypoint)
        
        # Share the waypoint with Drone 2
        with shared_data["lock"]:
            shared_data["waypoint"] = waypoint
        send_signal(1)  # Signal Drone 2 to proceed

    elif target_system == 2:
        # Drone 2 waits for the signal and retrieves the waypoint
        wait_for_signal(1)
        with shared_data["lock"]:
            waypoint = shared_data["waypoint"]
        print(f"Drone {target_system} received waypoint: {waypoint}")
        move_to_waypoint(connection, target_system, *waypoint)

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