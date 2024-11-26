from pymavlink import mavutil


def print_drone_info(drone):
    """
    Continuously print all information received from the drone.
    Displays the message type and its contents.
    """
    print("Listening for all drone information (press Ctrl+C to stop)...")
    try:
        while True:
            # Receive the next MAVLink message
            msg = drone.recv_match(blocking=True, timeout=1)
            if msg:
                # Print the message type and its content
                print(f"Message Type: {msg.get_type()}")
                print(msg.to_dict())  # Converts message data to a dictionary
                print("-" * 40)
            else:
                print("No message received.")
    except KeyboardInterrupt:
        print("Stopped listening for drone information.")
    except Exception as e:
        print(f"Error: {e}")


drone = mavutil.mavlink_connection('udpin:localhost:14550')
print_drone_info(drone)