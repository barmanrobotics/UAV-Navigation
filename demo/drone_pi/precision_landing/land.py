import cv2 # type: ignore
import time
from aruco_detect import connect_camera, detect_aruco_tags
from drone_control import send_velocity

def precision_landing(connection):
    """
    Execute precision landing using ArUco marker detection
    
    Args:
        connection: MAVLink connection object
    """
    # Configuration parameters
    USE_PI_CAMERA = True
    DESCEND_VELOCITY = 0.3
    KP = 0.5
    KD = 0.07
    KI = 0.0
    INITIAL_ERROR = 0.01
    
    # Connect to camera
    camera_source = connect_camera(USE_PI_CAMERA)
    print("Starting precision landing...")
    
    # Initialize PID variables
    i = 1
    previous_time = time.time()
    previous_error_x = 0.0
    previous_error_y = 0.0
    error_sum_x = 0.0
    error_sum_y = 0.0
    
    try:
        while True:
            frame = camera_source()
            if frame is None:
                print("No frame received.")
                continue
            
            position = detect_aruco_tags(frame)
            if position is not None:
                x, y, z = position
                error_x, error_y = y/100, x/100 

                # Calculate time delta
                current_time = time.time()
                if i == 1:
                    dt = 0.1
                    previous_error_x = error_x
                    previous_error_y = error_y
                    i = 2
                else:
                    dt = current_time - previous_time
                previous_time = current_time
                
                # PID calculation
                p_out_x, p_out_y = KP * error_x, KP * error_y
                d_out_x = KD * (error_x - previous_error_x) / dt
                d_out_y = KD * (error_y - previous_error_y) / dt
                
                error_sum_x += error_x * dt
                error_sum_y += error_y * dt

                i_out_x = KI * error_sum_x
                i_out_y = KI * error_sum_y

                previous_error_x = error_x
                previous_error_y = error_y

                # Calculate acceleration commands
                ax = p_out_x + d_out_x + i_out_x
                ay = p_out_y + d_out_y + i_out_y
                
                # Limit acceleration
                ax = max(min(ax, 0.6), -0.6)
                ay = max(min(ay, 0.6), -0.6)
                
                print(f"a_y: {ay} a_x: {ax}")
                
                # Adjust descent velocity based on altitude
                current_descent = 0.2 if z < 200 else DESCEND_VELOCITY
                
                # Send velocity command to drone
                send_velocity(connection, -ax, ay, current_descent)
            else:
                print("ArUco marker not detected, continuing descent")
                send_velocity(connection, 0, 0, 0.15)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Precision landing interrupted")
    finally:
        cv2.destroyAllWindows()
        print("Precision landing complete")

# For standalone testing
if __name__ == "__main__":
    import argparse
    from pymavlink import mavutil # type: ignore
    
    parser = argparse.ArgumentParser(description="UAV Precision Landing with ArUco Detection")
    parser.add_argument("--connection", type=str, default="udpin:localhost:14541", 
                        help="MAVLink connection string")
    args = parser.parse_args()
    
    # Create connection
    connection = mavutil.mavlink_connection(args.connection)
    connection.wait_heartbeat()
    print("Connected to the vehicle")
    
    # Execute precision landing
    precision_landing(connection)


