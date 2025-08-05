#Realsense HITL

from pymavlink import mavutil # type: ignore
import time
import math
import pymavlink.dialects.v20.all as dialect
import pyrealsense2 as rs
import cv2
import numpy as np

# Code from https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/opencv_pointcloud_viewer.py

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, rs.format.z16, 30)
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

# Start a connection listening to a UDP port
connection = mavutil.mavlink_connection('udpin:localhost:14550')

# # Wait for a heartbeat to confirm connection
connection.wait_heartbeat()
print("Connected to the vehicle")

# Start streaming
pipeline.start(config)

# Get stream profile and camera intrinsics
profile = pipeline.get_active_profile()


def dodge(vx,vy,vz,yaw):
    connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms, set to 0 as we are sending an instantaneous command
        connection.target_system, 
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # NED frame
        int(0b110111000111),  # type_mask to specify that we are setting velocity
        0,            # North (meters)
        0,            # East (meters)
        0,            # Down (meters)
        vx,            # velocity_x (m/s)
        vy,            # velocity_y (m/s)
        vz,            # velocity_z (m/s)
        0,            # acceleration_x (not used)
        0,            # acceleration_y (not used)
        0,            # acceleration_z (not used)
        yaw,            # yaw (not used)
        15             # yaw_rate (not used)
    )

# Get distance in front of depth sensor
dodging = False # track if currently dodging so we don't repeat commands


while True:
    # Grab camera data
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()

    depth_frame = frames.get_depth_frame()

    # depth_frame = decimate.process(depth_frame)
    w = depth_frame.get_width()
    h = depth_frame.get_height()

    dist = depth_frame.get_distance(int(w/2),int(h/2))

    msg = connection.recv_match(type='ATTITUDE', blocking=True)

    if not dodging:
        yaw = msg.yaw

    # print(dist, end="\r")
    print(f"{yaw}", end="\r")
    if dist < .5 and not dodging:
        dodge(0,5,0,0)
        dodging = True
    elif dodging:
        dodge(0,0,0,0)
        dodging = False # once done dodging, de-active dodge trigger
        yaw = msg.yaw

    time.sleep(.5)



    # yaw error, yaw keeps changing idk 