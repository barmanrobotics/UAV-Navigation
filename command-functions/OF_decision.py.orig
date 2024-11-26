#!/usr/bin/env python3
import math
import cv2 as cv
import numpy as np

def choose_direction(vect):
    scale = 1
    flow_threshold=60
    hor_threshold=35*scale
    ver_threshold=50*scale
    right_sum, left_sum, up_sum, down_sum = vect

    # Check if the overall flow is below the flow_threshold
    #total_flow = right_sum + left_sum + up_sum + down_sum
    total_flow = right_sum + left_sum
    print(total_flow, vect)
    if total_flow < flow_threshold:
        decision = "Move forward"
    elif right_sum > left_sum and abs(right_sum - left_sum) > hor_threshold:
        decision = "Move left"
    elif left_sum > right_sum and abs(left_sum - right_sum) > hor_threshold:
        decision = "Move right"
    # elif up_sum > down_sum and abs(up_sum - down_sum) > ver_threshold:
    #     decision = "move down"
    # elif down_sum > up_sum and abs(down_sum - up_sum) > ver_threshold:
    #     decision = "move up"
    else:
        decision = "stop"
    
    print(decision)
    return decision

def calculate_dir(fx, fy, x, y, width, height):
    right_sum = 0
    left_sum = 0
    up_sum = 0
    down_sum = 0

    ang = math.atan2(fy, fx)
    mag = math.sqrt(fx**2 + fy**2)

    # # Position-based scaling factor
    # position_factor = (width - x) / width  # Closer to right side gets smaller factor

    # # Right: -π/4 to π/4
    # if -0.25 * np.pi <= ang <= 0.25 * np.pi:
    #     right_sum += mag * position_factor  # Scale based on position

    # # Left: 3π/4 to -3π/4
    # elif ang >= 0.75 * np.pi or ang <= -0.75 * np.pi:
    #     left_sum += mag * (1 - position_factor)  # Opposite scaling for left

    # # Up: π/4 to 3π/4
    # elif 0.25 * np.pi < ang < 0.75 * np.pi:
    #     up_sum += mag

    # # Down: -3π/4 to -π/4
    # elif -0.75 * np.pi < ang < -0.25 * np.pi:
    #     down_sum += mag
    if x < (width / 2):
        left_sum += mag
    else:
        right_sum += mag
    
    if y < (height / 2):
        up_sum += mag
    else:
        down_sum += mag

    return [right_sum, left_sum, up_sum, down_sum]

def OF_cal(prvs, next_frame):
    height, width = prvs.shape[:2]

    # Calculate 2D optical flow
    flow = cv.calcOpticalFlowFarneback(prvs, next_frame, None, 0.5, 3, 15, 3, 5, 1.2, 0)

    # Convert the next_frame to a color image to draw colored arrows
    #arrow_frame = cv.cvtColor(next_frame, cv.COLOR_GRAY2BGR)
    arrow_frame = next_frame

    step_size = 200  # Spacing between the points where arrows are drawn
    scale = 10  # Scale for the arrow length

    direction_vector = np.array([0, 0, 0, 0], dtype=float)
    # Loop through the grid of points to draw arrows
    for y in range(0, height, step_size):
        for x in range(0, width, step_size):
            fx, fy = flow[y, x][0], flow[y, x][1]
            direction_vector += np.array(calculate_dir(fx, fy, x, y, width, height))
            if np.hypot(fx, fy) > 1.0:
                end_point = (int(x + fx * scale), int(y + fy * scale))
                cv.arrowedLine(arrow_frame, (x, y), end_point, (0, 255, 0), 1, tipLength=0.5)

    # Show the frame with arrows
    cv.imshow('Optical Flow Arrows', arrow_frame)
    cv.waitKey(1)  # This refreshes the display
    # Return the decision and update prvs
    return direction_vector

