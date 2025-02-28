import argparse
from aruco_detect import connect_camera, detect_aruco_tags
from drone_control import connect_drone, arm_disarm_drone, send_velocity, set_flight_mode, takeoff
import time
import cv2

# === Argument Parser ===
parser = argparse.ArgumentParser(description="UAV Precision Landing with ArUco Detection")
parser.add_argument("--use_pi_camera", action="store_true", help="Use Raspberry Pi Camera instead of simulation")
parser.add_argument("--descend_velocity", type=float, default=0.3, help="Set descend velocity (default: 0.3 m/s)")
parser.add_argument("--kp", type=float, default=0.5, help="Proportional gain for PID control")
parser.add_argument("--kd", type=float, default=0.07, help="Derivative gain for PID control")
parser.add_argument("--ki", type=float, default=0.0, help="Integral gain for PID control")
parser.add_argument("--connection", type=float, default=14541, help="Drone IP")
args = parser.parse_args()

USE_PI_CAMERA = args.use_pi_camera  # Set to False for simulation
descend_velocity = args.descend_velocity
kp, kd, ki = args.kp, args.kd, args.ki
previous_time = time.time()

# Connect to the drone
connection = connect_drone(args.connection)
set_flight_mode(connection, "GUIDED")
arm_disarm_drone(connection, 1)
takeoff(connection, 4)

# Connect to the camera (Pi Camera or Simulation)
camera_source = connect_camera(USE_PI_CAMERA)

i = 1
INITIAL_ERROR = 0.01
previous_time = time.time()
previous_error_x = 0.0
previous_error_y = 0.0
error_sum_x = 0.0
error_sum_y = 0.0

while True:
    frame = camera_source()
    if frame is None:
        print("No frame received.")
        continue
    
    position = detect_aruco_tags(frame)
    if position is not None:
        x, y, z = position
        error_x, error_y = y/100, x/100 

        current_time = time.time()
        if i==1:
            dt = 0.1
            previous_error_x = error_x
            previous_error_y = error_y
        else:
            dt = current_time - previous_time
        previous_time = current_time
        p_out_x, p_out_y = kp * error_x, kp * error_y
        d_out_x, d_out_y = kd * (error_x - previous_error_x) / dt, kd * (error_y - previous_error_y) / dt
        
        error_sum_x += error_x*dt
        error_sum_y += error_y*dt

        i_out_x = ki*error_sum_x
        i_out_y = ki*error_sum_y

        previous_error_x = error_x
        previous_error_y = error_y

        ax, ay = p_out_x + d_out_x + i_out_x, p_out_y + d_out_y + i_out_y
        if ax<-0.6:
            ax=-0.6
        if ax>0.6:
            ax=0.6
        if ay<-0.6:
            ay=-0.6
        if ay>0.6:
            ay=0.6
        print (f"a_y: {ay} a_x: {ax}")
        print("")        
        send_velocity(connection, -ax, ay, descend_velocity)
    else:
        send_velocity(connection, 0, 0, 0.15)
    
    i = i+1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
arm_disarm_drone(connection, 0)

