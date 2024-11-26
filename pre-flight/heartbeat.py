from pymavlink import mavutil

def check_heartbeat(drone, timeout=5):
    """Check for heartbeat messages within a specified timeout."""
    print("Checking heartbeat...")
    try:
        msg = drone.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg:
            print(f"Heartbeat received: {msg.to_dict()}")
            return True
        else:
            print("Heartbeat not received within timeout.")
            return False
    except Exception as e:
        print(f"Error checking heartbeat: {e}")
        return False


drone = mavutil.mavlink_connection('udpin:localhost:14550')
check_heartbeat(drone)