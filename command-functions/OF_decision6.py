#!/usr/bin/env python3
import time
import cv2
import numpy as np

# Initialize variables
MAX_COUNT = 100
need_to_init = True
night_mode = False
right_sum = left_sum = up_sum = down_sum = 0
points_prev = np.array([], dtype=np.float32)
points_next = np.array([], dtype=np.float32)
status = []
previous_decision = None
frame_gray_prev = None
right_sum = 0
left_sum = 0
up_sum = 0
down_sum = 0
middle_sum = 0

# Shi-Tomasi Parameters
shitomasi_params = dict(maxCorners=1000, qualityLevel=0.001, minDistance=1)

# Lucas-Kanade Parameters
lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

def draw_flow_vectors(prev_pts, next_pts, status, frame):
    global right_sum, left_sum, up_sum, down_sum, middle_sum
    total_count = 0

    if len(prev_pts) == 0 or len(next_pts) == 0:
        print("No points to process.")
        return

    frame = frame.copy()  # Ensure a writable copy of the frame

    for i, (p, q) in enumerate(zip(prev_pts, next_pts)):
        if not status[i]:
            continue

        # Convert points to integer tuples
        p = tuple(map(int, p.ravel()))
        q = tuple(map(int, q.ravel()))

        # Calculate angle and magnitude of flow
        angle = np.arctan2(p[1] - q[1], p[0] - q[0])
        hypotenuse = np.sqrt((p[0] - q[0])**2 + (p[1] - q[1])**2)

        # Scale the arrow vector
        q = (int(p[0] - 3 * hypotenuse * np.cos(angle)),
             int(p[1] - 3 * hypotenuse * np.sin(angle)))

        # Draw the flow vector
        cv2.arrowedLine(frame, p, q, (255, 255, 255), 1, tipLength=0.5)

        # Calculate magnitude
        mag = np.sqrt((q[0] - p[0])**2 + (q[1] - p[1])**2)
        w = frame.shape[1]

        # Categorize flow based on position
        if q[0] <  w // 3:  # Left third of the frame
            right_sum += mag
            total_count += 1
        elif q[0] > 2 * w // 3:  # Right third of the frame
            total_count += 1
            left_sum += mag
        else:  # Middle third of the frame
            middle_sum += mag
            total_count += 1

        # # Categorize flow based on position
        # if q[0] < w // 2:  # Left half of the frame
        #     right_sum += mag
        #     total_count += 1
        # else: # Right half of the frame
        #     total_count += 1
        #     left_sum += mag
        # if w // 3 < q[0] < 2* w // 3:
        #     middle_sum += mag
        #     total_count += 1
    
    if total_count != 0:
        right_sum /= total_count
        left_sum /= total_count
        middle_sum /= total_count

    # Debugging output
    print(f"Left Flow: {left_sum:.2f}, Mid Flow: {middle_sum:.2f}, Right Flow: {right_sum:.2f}")
    print(f"Up Flow: {up_sum:.2f}, Down Flow: {down_sum:.2f}")
    print("..........................................")



def make_decision():
    """
    Make decisions based on dense optical flow in left, middle, and right regions.
    """
    global previous_decision  # Use a global variable to track the previous decision
    threshold = 2   # Minimum difference to trigger movement
    fw_threshold = 2

    if fw_threshold > middle_sum:
        decision = "Move forward"
    elif right_sum > left_sum and abs(right_sum-left_sum) > threshold:
        decision = "Move right"
    elif left_sum > right_sum and abs(left_sum-right_sum) > threshold:
        decision = "Move left"
    else:
        decision = previous_decision

    # Update the previous decision
    previous_decision = decision

    return decision


def process_frame(frame):
    global points_prev, points_next, status, need_to_init, night_mode, frame_gray_prev

    REINIT_THRESHOLD = 600  # Minimum number of points before reinitializing

    # Convert the frame to grayscale
    #frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame_gray = frame
    mask = np.zeros_like(frame_gray)
    cv2.rectangle(mask, (200, 1), (450, 300), 255, -1)  # Define ROI

    if night_mode:
        frame[:] = 0

    if need_to_init:
        points_next = cv2.goodFeaturesToTrack(frame_gray, mask=None, **shitomasi_params)
        if points_next is not None:
            points_next = np.array(points_next, dtype=np.float32)
        need_to_init = False
    elif points_prev is not None and len(points_prev) > 0:
        points_next, status, _ = cv2.calcOpticalFlowPyrLK(frame_gray_prev, frame_gray, points_prev, None)
        if points_next is not None:
            draw_flow_vectors(points_prev, points_next, status, frame)

        # Make movement decision based on flow
        make_decision()

        # Keep only valid points
        points_prev = points_next[status.flatten() == 1] if points_next is not None else []

        # Check if reinitialization is needed
        if len(points_prev) < REINIT_THRESHOLD:
            print("Reinitializing tracking points...")
            need_to_init = True

    # Update points for the next iteration
    points_prev = points_next if points_next is not None else []
    frame_gray_prev = frame_gray.copy()
    cv2.imshow("OF", frame)
    cv2.waitKey(1)

    return previous_decision, middle_sum



