from pymavlink import mavutil
import time

def check_data_stream(drone, stream_id, rate=2, timeout=10):
    """Request and verify a data stream."""
    print(f"Requesting data stream: ID {stream_id} at {rate}Hz...")
    try:
        drone.mav.request_data_stream_send(
            drone.target_system, drone.target_component,
            stream_id, rate, 1  # Enable stream
        )
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = drone.recv_match(blocking=True, timeout=1)
            if msg and msg.get_type() != 'HEARTBEAT':  # Exclude heartbeats
                print(f"Data stream message received: {msg.to_dict()}")
                return True
        print("No data stream messages received.")
        return False
    except Exception as e:
        print(f"Error requesting data stream: {e}")
        return False


drone = mavutil.mavlink_connection('udpin:localhost:14550')
check_data_stream(drone, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS)