from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for the first heartbeat

#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


#set mode to GUIDED (4)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4, 0, 0, 0, 0, 0)
# Print acknowledgement of command
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

# Arm motors
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# Print acknowledgement of command
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

# Takeoff (10 meters)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

# Print acknowledgement of command
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)