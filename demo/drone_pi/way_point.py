import pymavlink.mavutil as mavutil # type: ignore

TARGET_SPEED = 0.5

def way_point(connection, target_x, target_y, target_z):
    try:
        # print(f"Moving to relative position: x={x_offset}m, y={y_offset}m, z={z_offset}m")
        print(f"Target position: x={target_x}m, y={target_y}m, z={target_z}m")

        connection.mav.command_long_send(
            connection.target_system, connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,
            4, # Mode 4 (guided)
            0, 0, 0, 0, 0
        )

        connection.mav.set_position_target_local_ned_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            target_x, target_y, target_z,
            TARGET_SPEED, TARGET_SPEED, TARGET_SPEED,
            0, 0, 0,
            0, 0
        )

        return True
    except Exception as e:
        print(f"Error in waypoint navigation: {e}")
        return False

if __name__ == "__main__":
    # This allows the script to be run directly for testing
    import sys
    if len(sys.argv) != 4:
        print("Usage: python3 way_point.py x_offset y_offset z_offset")
        sys.exit(1)
    
    connection = mavutil.mavlink_connection('udpin:localhost:14550')
    connection.wait_heartbeat()
    x, y, z = map(float, sys.argv[1:])
    way_point(connection, x, y, z)
