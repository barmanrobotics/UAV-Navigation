# Get live GPS and heading info from the drone

from pymavlink import mavutil # type: ignore
import time
import math
import pymavlink.dialects.v20.all as dialect


# Start a connection listening to a UDP port
connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")



UP = "\x1B[5A"
CLR = "\x1B[0K"

def get_gps_info():
    msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    
    # Extract latitude, longitude, and altitude from the message
    if msg:
        # print(f"{UP}lat: {msg.lat/1e7}{CLR}\n lon: {msg.lon/1e7}{CLR} \n alt: {msg.lat}{CLR} \n rel_alt: {msg.relative_alt}{CLR} \n heading: {msg.hdg}{CLR}")
        print(f"{msg.hdg} {msg.hdg*math.pi/180/100}                ", end="\r")

def continuous_readout():
    while True:
        get_gps_info()
        time.sleep(.1)

print("Printing GPS data\n------------------------\n\n\n\n\n")
continuous_readout()
