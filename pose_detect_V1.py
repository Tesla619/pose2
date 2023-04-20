import cv2
import numpy as np
#from cv2 import aruco

# Initialize the camera
cap = cv2.VideoCapture(0)

# Load the dictionary and parameters
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000) #cv2.aruco.DICT_6X6_250
#parameters = cv2.aruco.DetectorParameters_create()
parameters = cv2.aruco.DetectorParameters()

# Define the size of the markers (in meters)
marker_size = 0.05

# Define the camera matrix and distortion coefficients
camera_matrix = np.array([[ 6.73172250e+02, 0.00000000e+00, 3.21652381e+02],
                 [ 0.00000000e+00, 6.73172250e+02, 2.40854103e+02],
                 [ 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distortion_coefficients = np.array([-2.87888863e-01,  9.67075352e-02,  1.65928771e-03, -5.19671229e-04, -1.30327183e-02])

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the frame
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    # Estimate the pose of the markers
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coefficients)

    # If markers are detected, estimate their pose
    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix.reshape((3, 3)), distortion_coefficients)

        # Draw the axes of the markers
        for i in range(len(ids)):
            cv2.aruco.drawAxis(frame, camera_matrix.reshape((3, 3)), distortion_coefficients, rvecs[i], tvecs[i], marker_size)

    # Display the frame
    cv2.imshow('frame', frame)

    # Check for the 'q' key to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close the window
cap.release()
cv2.destroyAllWindows()
