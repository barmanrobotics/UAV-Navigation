from pymavlink import mavutil
import time

def connect_drone(port):
    connection = mavutil.mavlink_connection(f'udpin:localhost:{port}')
    connection.wait_heartbeat()
    print("Connected to the vehicle")
    return connection

def arm_disarm_drone(connection, value):
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, value, 0, 0, 0, 0, 0, 0
    )
    print("Arming..." if value else "Disarming...")
    time.sleep(2)

def takeoff(connection, altitude):
    print(f"Taking off to {altitude} meters...")
    connection.mav.command_long_send(
        connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            print("No altitude data received. Retrying...")
            continue
        current_alt = msg.relative_alt / 1000.0
        print(f"Current altitude: {current_alt:.2f} meters")
        if current_alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.5)

def set_flight_mode(connection, mode):
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return

    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg is None:
            print("No heartbeat received. Retrying...")
            continue
        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode:
            print(f"Mode changed to {mode}")
            break
        time.sleep(1)

def send_velocity(connection, ax, ay, vz):
    connection.mav.send(
        mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b110000000000,  # Ignore position, only use velocity
            0, 0, 0,  # Position x, y, z (ignored)
            0, 0, vz,  # Velocity x, y, z
            ax, ay, 0,  # Acceleration (ignored)
            0, 0  # Yaw, yaw rate (ignored)
        )
    )


