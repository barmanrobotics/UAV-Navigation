from pymavlink import mavutil
import time

def avoid_collision(connection):
    print("Executing AVOID maneuver: Ascending smoothly...")

    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_alt = msg.relative_alt / 1000.0

    target_alt = current_alt + 5
    print(f"Current Altitude: {current_alt} meters")
    print(f"Ascending to: {target_alt} meters")

    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, -1,
        0, 0, 0,
        0, 0
    )

    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.2f} meters")
        if current_alt >= target_alt - 0.5:
            break

    time.sleep(2)

    print(f"Return to orig alt: {current_alt - 5:.2f} m")

    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 1,
        0, 0, 0,
        0, 0
    )

    while True:
        msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.2f} meters")
        if current_alt <= target_alt - 4.5:
            break

    connection.mav.set_position_target_local_ned_send(
        0, connection.target_system, connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    print("Done")
