#!/usr/bin/env python3
# Import necessary libraries
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random

# Initialize a global variable to store the converted image
cv_image = None
# Create a CvBridge object for converting between ROS and OpenCV image formats
bridge = CvBridge()

# Function to transform coordinates from image space to real-world space
def transformCoords(xc, yc, height, width):
    real_x = (0.8 - (xc * 0.8) / width)
    real_y = ((yc * 1.5) / height)
    real_x = (real_x - (((width * 0.2625) * 0.8) / width)) * 1000
    real_y = (real_y - (((height / 2) * 1.5) / height)) * 1000

    return real_x, real_y

# Function to improve the contrast of an image using CLAHE
def improve_contrast(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return image

# Callback function to process incoming image messages
def image_callback(msg):
    global cv_image
    try:
        # Convert the ROS Image message to a format that OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Rotate the image 90 degrees counterclockwise
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # Improve the contrast of the image
        cv_image = improve_contrast(cv_image)
        # Convert the image to HSV color space
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
    except CvBridgeError as e:
        # Log an error if the conversion fails
        rospy.logerr(e)

# Function to detect objects in the image and publish their coordinates
def detect_objects_and_publish():
    try:
        global cv_image
        if cv_image is None:
            return

        # Copy the HSV image for processing
        hsv_image = cv_image.copy()

        # Convert the image to grayscale
        maskT = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2GRAY)
        # Apply erosion and dilation to remove noise
        kernel = np.ones((5, 5), np.uint8)
        maskT = cv2.erode(maskT, kernel, iterations=3)
        maskT = cv2.dilate(maskT, kernel, iterations=3)

        # Apply binary thresholding
        _, maskT = cv2.threshold(maskT, 60, 255, cv2.THRESH_BINARY)

        # Apply Canny edge detection
        maskT = cv2.Canny(maskT, 100, 200)

        # Display the processed image
        cv2.imshow("ImageKinect1: finish", maskT)
        cv2.waitKey(3)

        # Find contours in the processed image
        contours, _ = cv2.findContours(maskT, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize a list to store the detected object information
        vector_string = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter contours based on area
            if 8000 > area > 5000:
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate the coordinates of the rectangle's corners
                x1, y1 = x, y
                x2, y2 = x + w, y
                x3, y3 = x + w, y + h
                x4, y4 = x, y + h

                # Calculate the center of the rectangle
                xc = (w / 2) + x
                yc = (h / 2) + y

                # Get the dimensions of the image
                height, width = hsv_image.shape[:2]

                # Calculate the center coordinates for transformation
                x1_centro = (x1 + xc) / 2
                x2_centro = (x3 + xc) / 2
                y1_centro = (y1 + yc) / 2
                y2_centro = (y3 + yc) / 2

                centro1 = [x1_centro, x2_centro, xc]
                centro2 = [y1_centro, y2_centro, yc]

                # Randomly choose one of the center coordinates
                xcoord = random.choice(centro1)
                ycoord = random.choice(centro2)

                # Transform the coordinates to real-world space
                real_x, real_y = transformCoords(xcoord, ycoord, height, width)

                # Determine the color of the object based on its hue value
                color_f = ""

                object_center = cv_image[int(yc), int(xc)]
                hue_value = object_center[0]

                if 0 < hue_value <= 20 or 160 <= hue_value <= 180:
                    color_f = "red"
                elif 30 <= hue_value < 89:
                    color_f = "green"
                elif 90 <= hue_value <= 153:
                    color_f = "blue"
                else:
                    color_f = "unknown"

                # Filter out invalid coordinates and unknown colors
                if real_y == -750.0 or real_x == 560.0 or color_f == "unknown":
                    pass
                else:
                    vector_string.append(f"{color_f}({real_x},{real_y}")

        # Publish the detected object information
        meta_coords_pub.publish(str(vector_string))

        rate.sleep()
    except:
        pass

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("detectMat", anonymous=True)
    rate = rospy.Rate(1)
    # Subscribe to the image topic
    rospy.Subscriber('/tableColor', Image, image_callback)
    # Create a publisher for the object coordinates
    meta_coords_pub = rospy.Publisher('/matCoords', String, queue_size=10)

    # Main loop to continuously process images and detect objects
    while not rospy.is_shutdown():
        try:
            detect_objects_and_publish()
        except Exception as e:
            rospy.logerr(e)
            pass
#!/usr/bin/env python3
# Import necessary libraries
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import random

# Initialize a global variable to store the converted image
cv_image = None
# Create a CvBridge object for converting between ROS and OpenCV image formats
bridge = CvBridge()

# Function to transform coordinates from image space to real-world space
def transformCoords(xc, yc, height, width):
    real_x = (0.8 - (xc * 0.8) / width)
    real_y = ((yc * 1.5) / height)
    real_x = (real_x - (((width * 0.2625) * 0.8) / width)) * 1000
    real_y = (real_y - (((height / 2) * 1.5) / height)) * 1000

    return real_x, real_y

# Function to improve the contrast of an image using CLAHE
def improve_contrast(image):
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return image

# Callback function to process incoming image messages
def image_callback(msg):
    global cv_image
    try:
        # Convert the ROS Image message to a format that OpenCV can work with
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        # Rotate the image 90 degrees counterclockwise
        cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        # Improve the contrast of the image
        cv_image = improve_contrast(cv_image)
        # Convert the image to HSV color space
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
    except CvBridgeError as e:
        # Log an error if the conversion fails
        rospy.logerr(e)

# Function to detect objects in the image and publish their coordinates
def detect_objects_and_publish():
    try:
        global cv_image
        if cv_image is None:
            return

        # Copy the HSV image for processing
        hsv_image = cv_image.copy()

        # Convert the image to grayscale
        maskT = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2GRAY)
        # Apply erosion and dilation to remove noise
        kernel = np.ones((5, 5), np.uint8)
        maskT = cv2.erode(maskT, kernel, iterations=3)
        maskT = cv2.dilate(maskT, kernel, iterations=3)

        # Apply binary thresholding
        _, maskT = cv2.threshold(maskT, 60, 255, cv2.THRESH_BINARY)

        # Apply Canny edge detection
        maskT = cv2.Canny(maskT, 100, 200)

        # Display the processed image
        cv2.imshow("ImageKinect1: finish", maskT)
        cv2.waitKey(3)

        # Find contours in the processed image
        contours, _ = cv2.findContours(maskT, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Initialize a list to store the detected object information
        vector_string = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter contours based on area
            if 8000 > area > 5000:
                x, y, w, h = cv2.boundingRect(contour)

                # Calculate the coordinates of the rectangle's corners
                x1, y1 = x, y
                x2, y2 = x + w, y
                x3, y3 = x + w, y + h
                x4, y4 = x, y + h

                # Calculate the center of the rectangle
                xc = (w / 2) + x
                yc = (h / 2) + y

                # Get the dimensions of the image
                height, width = hsv_image.shape[:2]

                # Calculate the center coordinates for transformation
                x1_centro = (x1 + 70 + xc) / 2
                x2_centro = (x3 - 70 + xc) / 2
                y1_centro = (y1 - 70 + yc) / 2
                y2_centro = (y3 + 70 + yc) / 2

                centro1 = [x1_centro, x2_centro, xc]
                centro2 = [y1_centro, y2_centro, yc]

                # Randomly choose one of the center coordinates
                xcoord = random.choice(centro1)
                ycoord = random.choice(centro2)

                # Transform the coordinates to real-world space
                real_x, real_y = transformCoords(xcoord, ycoord, height, width)

                # Determine the color of the object based on its hue value
                color_f = ""

                object_center = cv_image[int(yc), int(xc)]
                hue_value = object_center[0]

                if 0 < hue_value <= 20 or 160 <= hue_value <= 180:
                    color_f = "red"
                elif 30 <= hue_value < 89:
                    color_f = "green"
                elif 90 <= hue_value <= 153:
                    color_f = "blue"
                else:
                    color_f = "unknown"

                # Filter out invalid coordinates and unknown colors
                if real_y == -750.0 or real_x == 560.0 or color_f == "unknown":
                    pass
                else:
                    vector_string.append(f"{color_f}({real_x},{real_y}")

        # Publish the detected object information
        meta_coords_pub.publish(str(vector_string))

        rate.sleep()
    except:
        pass

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node("detectFinish", anonymous=True)
    rate = rospy.Rate(1)
    # Subscribe to the image topic
    rospy.Subscriber('/tableColor', Image, image_callback)
    # Create a publisher for the object coordinates
    meta_coords_pub = rospy.Publisher('/matCoords', String, queue_size=10)

    # Main loop to continuously process images and detect objects
    while not rospy.is_shutdown():
        try:
            detect_objects_and_publish()
        except Exception as e:
            rospy.logerr(e)
            pass
