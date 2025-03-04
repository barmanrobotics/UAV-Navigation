import pymavlink.mavutil as mavutil # type: ignore

def way_point(connection, x_offset, y_offset, z_offset):
    try:
        print(f"Moving to relative position: x={x_offset}m, y={y_offset}m, z={z_offset}m")
        
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_x = msg.x
        current_y = msg.y
        current_z = msg.z
        
        target_x = current_x + x_offset
        target_y = current_y + y_offset
        target_z = current_z + z_offset
        
        print(f"Target position: x={target_x}m, y={target_y}m, z={target_z}m")
        
        connection.mav.set_position_target_local_ned_send(
            0,
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            int(0b110111111000),
            target_x, target_y, -target_z,
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            0, 0      # yaw, yaw_rate
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
    
    
