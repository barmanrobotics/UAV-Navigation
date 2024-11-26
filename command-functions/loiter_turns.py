# Owen Bartlett
# This function sets the drone to loiter with turns for a certain amount of time at a specific altitude
# I wrote this and am still not sure how it's any different from loiter with time
# MAY BE FAULTY IMPLEMENTATION
import time
import math

connection = mavutil.mavlink_connection('udpin:localhost:14550')

# Wait for a heartbeat to confirm connection
connection.wait_heartbeat()

def loiter_for_turns(turns, radius, altitude):
    """
    Make the drone loiter for a specified number of turns around the current position.

    :param turns: Number of turns to make during loitering
    :param radius: Radius of the loitering circle (in meters)
    :param altitude: Altitude to maintain during loitering (in meters)
    """
    print(f"Loitering for {turns} turns with radius {radius} meters at altitude {altitude} meters.")

    # Send MAVLink loiter command (MAV_CMD_NAV_LOITER_TURNS)
    connection.mav.command_long_send(
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS,  # Loiter command for turns
        0,  # Confirmation (set to 0)
        turns,  # Param 1: Number of turns to make
        radius,  # Param 2: Radius of the loitering circle (in meters)
        altitude,  # Param 3: Altitude to loiter at (in meters)
        0,  # Param 4: Reserved (set to 0)
        0,  # Param 5: Reserved (set to 0)
        0,  # Param 6: Reserved (set to 0)
        0   # Param 7: Reserved (set to 0)
    )

    # Wait for the loitering to finish
    start_time = time.time()
    while True:
        # Here you might want to monitor progress, but for simplicity, we wait for a timeout
        elapsed_time = time.time() - start_time
        print(f"Loitering... Elapsed time: {elapsed_time:.2f} seconds")
        
        # You can set a timeout to stop waiting after a certain period
        if elapsed_time > (turns * 20):  # Approximate time per turn (20 seconds per turn)
            print("Loitering complete.")
            break

        time.sleep(1)