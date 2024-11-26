from pymavlink import mavutil
import time


def check_message_round_trip(drone):
    """Send a test message and verify round-trip response."""
    print("Testing message round-trip...")
    try:
        # Define a test message
        test_name = "TEST"  # Must be <= 10 characters for NAMED_VALUE_FLOAT
        test_value = 123.45

        # Send the test message
        print(f"Sending test message: {test_name} = {test_value}")
        drone.mav.named_value_float_send(
            drone.target_system,         # Target system
            int(time.time()),            # Timestamp (seconds since epoch)
            test_name,                   # Name (max 10 characters)
            test_value                   # Float value
        )
        
        # Wait for a response
        while True:
            msg = drone.recv_match(blocking=True, timeout=5)
            if not msg:
                print("No message received within the timeout.")
                return False
            
            # Print the raw message for debugging
            print(f"Received message: {msg}")

            # Check if the message type is NAMED_VALUE_FLOAT
            if msg.get_type() == 'NAMED_VALUE_FLOAT':
                # Ensure the name is processed correctly
                if hasattr(msg, 'name'):
                    name = msg.name  # Access the name attribute
                    if isinstance(name, (bytes, str)):  # Handle byte strings or strings
                        name = name.decode('utf-8').strip() if isinstance(name, bytes) else name.strip()
                        if name == test_name:
                            print(f"Round-trip test successful: {msg.to_dict()}")
                            return True
                        else:
                            print(f"Received message with unmatched name: {name}")
                    else:
                        print(f"Unexpected type for name: {type(name)}")
                else:
                    print("NAMED_VALUE_FLOAT message does not contain a 'name' attribute.")
            else:
                print(f"Ignoring unrelated message type: {msg.get_type()}")

    except Exception as e:
        print(f"Error during round-trip test: {e}")
        return False

    

drone = mavutil.mavlink_connection('udpin:localhost:14550')
check_message_round_trip(drone)