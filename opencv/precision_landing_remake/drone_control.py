import time
from pymavlink import mavutil

def connect_drone():
    connection = mavutil.mavlink_connection('udpin:localhost:14541')
    connection.wait_heartbeat()
    print("Connected to the vehicle")
    return connection

def arm_disarm_drone(connection, value):
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, value, 0, 0, 0, 0, 0, 0
    )
    if value == 1:
        print("Arming...")
        while True:
            heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
            if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print("Drone is armed")
                break
            time.sleep(0.5)
    else:
        print("Disarming...")
        msg = connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)

def set_flight_mode(connection, mode):
    mode_id = connection.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return

    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0
    )
    
    while True:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True)
        current_mode = mavutil.mode_string_v10(msg)
        if current_mode == mode:
            print(f"Mode changed to {mode}")
            break
        time.sleep(1)

def send_velocity(connection, vx, vy, vz):
    connection.mav.set_position_target_local_ned_send(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b110111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
