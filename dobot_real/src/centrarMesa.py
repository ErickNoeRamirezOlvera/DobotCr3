#!/usr/bin/env python3
# Import necessary libraries
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from dobot_real.msg import coords

# Initialize a global variable to store the converted image
cv_image = None

# Define the callback function to handle incoming image messages
def image_callback(msg):
    global cv_image
    try:
        # Convert the ROS Image message to a format that OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        # Log an error if the conversion fails
        rospy.logerr(f"CV Bridge Error: {e}")

if __name__ == '__main__':
    # Initialize the CvBridge object
    bridge = CvBridge()
    
    # Initialize the ROS node with a unique name
    rospy.init_node("centrarMesa", anonymous=True)
    # Subscribe to the camera's image topic
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

    # Main loop to process the images
    while not rospy.is_shutdown():
        try:
            # If an image has been received and converted
            if cv_image is not None:
                # Get the dimensions of the image
                height, width = cv_image.shape[:2]

                # Calculate the coordinates for the center of the image
                x = (width // 2)
                y = height // 2

                # Draw a circle at the center of the image
                cv2.circle(cv_image, (x, y), 10, (255, 0, 0), 2)

                # Calculate the coordinates for a rectangle around the center
                xLineIZQ = int(x * 0.23)
                yLineIZQ = int(y * 0.17474)
                xLineDER = int(x * 1.77)
                yLineDER = int(y * 1.2526)

                top_left = (xLineIZQ, yLineIZQ)
                bottom_right = (xLineDER, yLineDER)

                # Draw the rectangle on the image
                cv2.rectangle(cv_image, top_left, bottom_right, (255, 0, 0), 2)

                # Display the image in a window named "Calibration"
                cv2.imshow("Calibration", cv_image)
                cv2.waitKey(3)

        except Exception as e:
            # Log any errors that occur in the main loop
            rospy.logerr(f"Error in main loop: {e}")
            pass
