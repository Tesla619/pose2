import socket
import time
import sys
import cv2

# Define the port on which you want to connect
port = 12345

# Setting up the counter
counter = 1

# Importing the background image
image = cv2.imread("image.jpg")

# Get the dimensions of the image
height, width, _ = image.shape

# Choose the font, scale, and color
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 5
color = (0, 0, 0)  # White color

def output(image, counter):
    # Define the text to display
    text = f"{counter}"    
    
    # Calculate the text size
    text_size, _ = cv2.getTextSize(text, font, scale, 2)
    
    # Calculate the text position
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2
    
    # Copy the image for display purposes
    image_copy = image.copy()
    
    # Add the text to the image
    cv2.putText(image_copy, text, (text_x, text_y), font, scale, color, 2, cv2.LINE_AA)
    
    # Display Image
    cv2.imshow("Data Counter", image_copy)

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# connect to the server on local computer
s.bind(('localhost', port))

# put the socket into listening mode
s.listen(5)

# establish a connection with the client
c, addr = s.accept()

# print connection received from
print('Got connection from', addr)

while True:           
    # Define the terminator
    terminator = '\n'  # Newline character as the terminator

    # Convert the counter to a string and add the terminator
    data = str(counter) + terminator
    
    # Send the data
    c.send(data.encode())    
    #time.sleep(1)  # delay to simulate real-time data   
    
    output(image, counter)
    counter += 1 
        
    key = cv2.waitKey(1) & 0XFF
    if key == ord("q"):
        break

cv2.destroyAllWindows()
c.close()