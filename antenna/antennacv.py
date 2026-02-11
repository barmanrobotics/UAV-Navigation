import cv2
import numpy as np

cap = cv2.VideoCapture(0)  # Start webcam

# helpful utlity for getting color ranges
# https://pseudopencv.site/utilities/hsvcolormask/

# extract green points and find their center
# 2/3/26 - technically works, although very noisy and frequently loses green points

while True:
    _, frame = cap.read()
    
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range for blue color in HSVds
    lower_green = np.array([31,100,90])
    upper_green = np.array([89, 255, 255])

    # Create mask
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Filter the blue region
    result = cv2.bitwise_and(frame, frame, mask=mask)

    
    # kernel = np.ones((5,5), np.uint8)
    # output_img = cv2.dilate(output_img, kernel, iterations=1)
    blank = np.zeros(shape=frame.shape)
    output_img = blank.copy()


    contours, hier = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    points = []

    for c in contours:
        # get the center point of each contour
        M = cv2.moments(c)
        try:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            points.append((cx,cy))
            output_img = cv2.circle(output_img, (cx,cy), 2, (255,0,0),2)
        except:
            pass

    line_data = []
    intersections = []
    d = 10 # pixel range to consider overlap

    for i in range(len(points)):
        p1 = points[i]
        for j in range(i+1,len(points)):
            p2 = points[j]
            output_img = cv2.line(output_img,p1,p2,(0,255,0),2)
            x1,y1 = p1
            x2,y2 = p2
            try:
                m = (y2-y1) / (x2-x1)
                b = y1 - m * x1
                for data in line_data:
                    m2,b2 = data
                    x = (b2-b)/(m-m2)
                    y = int(m2*x+b2)
                    x = int(x)
                    print(m,b,x,y,end='                  \r')
                    if not ((x1 - d < x < x1 + d and y1 - d < y < y1 + d) or (x2 - d < x < x2 + d and y2 - d < y < y2 + d)):
                        intersections.append((x,y))
            except:
                pass
            line_data.append([m,b])

    for p in intersections:
        output_img = cv2.circle(output_img, p, 2, (255,255,255),2)

    # Show frames
    cv2.imshow('Original Frame', frame)
    # cv2.imshow('Green Filtered Result', output_img)
    cv2.imshow('lines', output_img)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break  # Exit the loop when 'q' is pressed

cap.release()
cv2.destroyAllWindows()
print("\nExiting...")  # Confirm exit