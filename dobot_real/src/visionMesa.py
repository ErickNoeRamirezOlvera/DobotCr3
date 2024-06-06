#!/usr/bin/env python3

# Import necessary ROS and Python libraries
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from dobot_real.msg import coords

# Initialize global variables
cv_imageColor = None
corners = coords()

# Function to crop a rectangle from an image
def crop_rectangle(image, center_x, center_y, width, height):
    x1 = int(center_x - width / 2)
    y1 = int(center_y - height / 2)
    x2 = int(center_x + width / 2)
    y2 = int(center_y + height / 2)
    cropped_image = image[y1:y2, x1:x2]
    return cropped_image

# Callback function to handle color image messages
def image_callback(msg):
    global cv_imageColor
    try:
        cv_imageColor = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(e)
        return

# Function to improve the contrast of an image
def improve_contrast(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return image

if __name__ == '__main__':
    # Initialize the CvBridge object
    bridge = CvBridge()
    
    # Initialize the ROS node
    rospy.init_node("visionMesa", anonymous=True)
    rate = rospy.Rate(1)  # Set rate to 1 Hz
    
    # Subscribe to the depth image and RGB image topics
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    
    # Initialize publishers for the processed images and coordinates
    tableColor_pub = rospy.Publisher("/tableColor", Image, queue_size=10)
    table_coords_pub = rospy.Publisher("/table_coordinates", coords, queue_size=10)

    while not rospy.is_shutdown():
        try:
            if cv_imageColor is not None:

                # Improve contrast of the color image
                img = cv_imageColor[25:315, 55:585]
                img = improve_contrast(img)
                
                # Convert image to HSV color space
                image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
                
                
                # Create a kernel for morphological operations
                kernel = np.ones((3, 3), np.uint8)
                
                # Perform erosion and dilation
                maskT = cv2.erode(image, kernel, iterations=3)
                maskT = cv2.dilate(maskT, kernel, iterations=3)
                
                # Apply binary threshold
                _, maskT = cv2.threshold(maskT, 110, 255, cv2.THRESH_BINARY)

                # Create a kernel for morphological operations
                kernel = np.ones((3, 3), np.uint8)

                # Perform erosion and dilation
                maskT = cv2.erode(maskT, kernel, iterations=5)
                maskT = cv2.dilate(maskT, kernel, iterations=5)

                maskT = cv2.cvtColor(maskT, cv2.COLOR_BGR2GRAY)
                
                _, maskT = cv2.threshold(maskT, 40, 255, cv2.THRESH_BINARY_INV)

                # Create a kernel for morphological operations
                kernel = np.ones((5, 5), np.uint8)

                # Perform erosion and dilation again
                maskT = cv2.erode(maskT, kernel, iterations=3)
                maskT = cv2.dilate(maskT, kernel, iterations=3)

                # Display the processed image
                cv2.imshow("ImageKinect3: table", maskT)
                cv2.waitKey(3)
                
                # Find contours in the binary image
                contours, _ = cv2.findContours(maskT, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Initialize variables for the bounding box
                x, y, w, h = 0, 0, 0, 0
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if 140000 > area > 100000:
                        x, y, w, h = cv2.boundingRect(contour)
                        
                        # Calculate corner coordinates
                        x1, y1 = x, y
                        x2, y2 = x + w, y
                        x3, y3 = x + w, y + h
                        x4, y4 = x, y + h
                        corners.x_coordinates = [x1, x2, x3, x4]
                        corners.y_coordinates = [y1, y2, y3, y4]
                
                # Calculate center coordinates of the bounding box
                xC = (w / 2) + x
                yC = (h / 2) + y
                
                # Crop the rectangle from the color image
                crop = crop_rectangle(img, xC, yC, w, h)
                
                # Black out a specified rectangle in the cropped image
                top_left = (152, 145)
                bottom_right = (335, 266)
                cv2.rectangle(crop, top_left, bottom_right, (0, 0, 0), -1)
                
                # Convert the cropped image back to ROS Image message
                crop = bridge.cv2_to_imgmsg(crop, encoding="bgr8")
                
                # Publish the coordinates and cropped image
                table_coords_pub.publish(corners)
                tableColor_pub.publish(crop)
                
                rate.sleep()
                
        except Exception as e:
            pass
