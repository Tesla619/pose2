import numpy as np
import cv2
import time
import math
#import matlab.engine

# Load the dictionary and parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
parameters = cv2.aruco.DetectorParameters_create()

# Define the size of the markers (in meters)
marker_size = 0.05

# Define the camera matrix and distortion coefficients
camera_matrix = np.array([[6.73172250e+02, 0.00000000e+00, 3.21652381e+02],
                          [0.00000000e+00, 6.73172250e+02, 2.40854103e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([-2.87888863e-01, 9.67075352e-02, 1.65928771e-03, -5.19671229e-04, -1.30327183e-02])

# Initialize the camera
cap = cv2.VideoCapture(0)

# Mapping of marker IDs to their corresponding centers
marker_centers = {}

# Define the IDs to move and the amount to move
ids_to_move = [4, 5, 8, 11]  # Add the desired marker IDs to move
#move_amount = (0, 0, 0,  0)  # FOR SPECIFIC AMOUNT TESTING #-110

# Define the connection options between markers
connections_options = [
    [(0, 4), (4,  8), ( 8, 12)],    # Front
    [(1, 5), (5,  9), ( 9, 13)],    # Left
    [(2, 6), (6, 10), (10, 14)],    # Back
    [(3, 7), (7, 11), (11, 15)]     # Right
]

# Initialize the selected option index
selected_option_index = 0

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    #frame = cv2.imread("input.png")

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    # If markers are detected, estimate their pose
    if ids is not None: #and len(ids) >= 2:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coefficients)

        # Draw the axes of the markers
        for i in range(len(ids)):
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.aruco.drawAxis(frame, camera_matrix, distortion_coefficients, rvecs[i], tvecs[i], marker_size)   
            
            print("rvecs: ", rvecs[i])
            print("tvecs: ", tvecs[i])
            print("ids: ", ids[i])
            print("corners: ", corners[i])
            print("marker size: ", marker_size)
            print("\n\n")
            
    # Display the frame
    cv2.imshow('frame', frame)

    # Check for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()

def line():
    # # Calculate the center points of the markers
    # marker_centers = {ids[i][0]: np.mean(corners[i][0], axis=0) for i in range(len(ids))}
    #
    # # Move the centers of the specified IDs by the specified amount
    # for id_to_move in ids_to_move:
    #     if id_to_move in marker_centers:
    #         marker_centers[id_to_move] += move_amount   
    #       
    # for connections in connections_options:
    #     for src_id, dest_id in connections:
    #         if src_id in marker_centers and dest_id in marker_centers:
    #             src_center = tuple(map(int, marker_centers[src_id].ravel()))
    #             dest_center = tuple(map(int, marker_centers[dest_id].ravel()))
    #             cv2.line(frame, src_center, dest_center, (0, 255, 0), thickness=5)    
    print()
