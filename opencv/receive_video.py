import socket
import cv2
import struct
import numpy as np
import time

# Server Configuration
HOST = '10.203.40.188'  # IP address of the server (Raspberry Pi)
PORT = 9999              # Same port as the server

# Create socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
print(f"Connected to server at {HOST}:{PORT}")

while True:
    # Receive the size of the frame
    frame_size_data = client_socket.recv(4)
    if len(frame_size_data) < 4:
        print("Error: Incomplete frame size data.")
        break
    
    frame_size = struct.unpack('!I', frame_size_data)[0]
    
    # Receive the frame data
    frame_data = b''
    while len(frame_data) < frame_size:
        packet = client_socket.recv(frame_size - len(frame_data))
        if not packet:
            print("Error: Lost connection to the server.")
            break
        frame_data += packet
    
    # Decode the frame data back into a numpy array
    nparr = np.frombuffer(frame_data, np.uint8)
    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    
    if frame is None:
        print("Error: Failed to decode frame.")
        break
    
    # Display the frame
    cv2.imshow('Video Stream', frame)
    #print("received")
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    #time.sleep(0.0084)

# Release resources
client_socket.close()
cv2.destroyAllWindows()
