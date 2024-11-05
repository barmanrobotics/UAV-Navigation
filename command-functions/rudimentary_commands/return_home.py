from pymavlink import mavutil
import time

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat

#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (the_connection.target_system, the_connection.target_component))

# Define position to fly to: 20 meters forward -10 meters down (10 meters above the ground)
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, 
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 0, 0, -10, 0, 0, 0, 0, 0, 0, 0, 0))


while 1:
    msg = the_connection.recv_match( type='NAV_CONTROLLER_OUTPUT', blocking=True)
    if msg.wp_dist == 0:
        #set mode to LAND (9)
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 9, 0, 0, 0, 0, 0)
        # Print acknowledgement of command
        msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
        print(msg)
        break

