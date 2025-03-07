from pymavlink import mavutil 

the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

# Wait for the first heartbeat
#  This sets the system and component ID of remot system for the link
the_connection.wait_heartbeat()
print("Heartbeat for system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

#CHANGE MODE TO GUIDED
mode_id = the_connection.mode_mapping()['GUIDED']

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)

#ARM
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)


#TAKEOFF
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 4)

msg = the_connection.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)
