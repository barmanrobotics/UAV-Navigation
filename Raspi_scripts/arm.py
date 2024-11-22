# Owen Bartlett
# Function to arm the motors

from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud = 57600) #if directly connecting to R pi via serial
#connection = mavutil.mavlink_connection('udpin:localhost:14551') #if using mavproxy and then connecting to its output port.


# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

#If directly connecting a script to the flight controller via serial without mavproxy, by default data streaming is not on.
#This means that if you'd like to access anything other than heartbeat, timesync and some parameter values, your code will
#be stuck in a while loop as you will receive "none" message
#Hence we initialize this by running this command

connection.mav.request_data_stream_send(
    connection.target_system, connection.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,10,1)


#arm drone
def arm_drone():
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Arming...")
    # Wait until armed
    while True:
        heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Drone is armed")
            break
        time.sleep(0.5)

#Print altitude
def alt():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    current_alt = msg.relative_alt / 1000.0
    print(f"Current altitude: {current_alt:.2f} meters")


arm_drone()
alt()
