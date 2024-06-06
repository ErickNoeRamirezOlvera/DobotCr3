#!/usr/bin/env python3

# Import necessary ROS and Python libraries
import rospy
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from dobot_real.msg import coords
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import cv2
import numpy as np
import statistics as st

# Initialize global variables
cv_imageColor = None
points = None
bridge = CvBridge()
xCoords = None
yCoords = None

# Callback function to update table coordinates from the coords message
def callback_tableCoords(msg):
    global xCoords, yCoords
    xCoords = msg.x_coordinates
    yCoords = msg.y_coordinates

# Callback function to process incoming color image messages
def image_callbackColor(msg):
    global cv_imageColor
    try:
        # Convert ROS Image message to OpenCV format
        cv_imageColor = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(e)
        return

# Callback function to process incoming point cloud messages
def callback_pointcloud(msg):
    global points
    try:
        # Update global points variable with incoming point cloud data
        points = msg
    except Exception as e:
        rospy.logerr(e)

# Function to improve the contrast of an image using CLAHE (Contrast Limited Adaptive Histogram Equalization)
def improve_contrast(image):
    # Convert BGR image to LAB color space
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    # Apply CLAHE to the L (lightness) channel
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    # Convert LAB image back to BGR color space
    image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
    return image

# Function to detect the color of an object at given coordinates
def colorDetection(xc, yc):
    try:
        color = ""
        # Rotate and enhance the contrast of the image
        image = cv2.rotate(cv_imageColor, cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = improve_contrast(image)
        # Convert image to HSV color space
        hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Check if the coordinates are within the image bounds
        if 0 <= yc < hsv_frame.shape[0] and 0 <= xc < hsv_frame.shape[1]:
            # Get the HSV value at the specified coordinates
            object_center = hsv_frame[int(yc), int(xc)]
            hue_value = object_center[0]
            # Determine color based on hue value
            if 0 < hue_value <= 20 or 160 <= hue_value <= 180:
                color = "red"
            elif 30 <= hue_value < 89:
                color = "green"
            elif 90 <= hue_value <= 153:
                color = "blue"
            else:
                color = "unknown"
        else:
            color = "unknown"

        return color
    
    except Exception as e:
        rospy.logerr(e)
        return "unknown"

def ed(xac, xdes, yac, ydes):
    dis = np.sqrt((xdes - xac)**2 + (ydes - yac)**2)
    return dis

# Function to get depth at a specific point in the point cloud
def get_depth_at(x, y):
    if points is not None:
        # Iterate over the point cloud to get the depth at the specified coordinates
        for point in pc2.read_points(points, field_names=("x", "y", "z"), skip_nans=True, uvs=[[x, y]]):
            return point[2]
    return None

# Function to compute the Z coordinate from the image and table coordinates
def coordZ(imagen , xdis, ydis, dis, areaXY):
    try:
        # Rotate the image for correct orientation
        img = cv2.rotate(imagen, cv2.ROTATE_90_CLOCKWISE)

        # Get the minimum table coordinates
        minTablex = min(xCoords)
        minTabley = min(yCoords)

        # Find contours in the image
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        x, y, w, h = 0, 0, 0, 0

        height, width = img.shape[:2]

        for contour in contours:
            area = cv2.contourArea(contour)
            if 250 < area < 1000:
                # Get the bounding box for the contour
                x, y, w, h = cv2.boundingRect(contour)

                xc = (w / 2) + x
                yc = (h / 2) + y

                xDobot = width // 2
                yDobot = int(height * 0.7474)

                xdist = abs(xc - xDobot)
                ydist = abs(yc - yDobot)

                dist = int(ed(xc, xDobot, yc, yDobot))
                
                if (ydis - 1) <= xdist <= (ydis + 1) and (xdis - 1) <= ydist <= (xdis + 1) and (dis - 1) <= dist <= (dis + 1) and (areaXY - 1) <= area <= (areaXY + 1):
                    # Calculate the center coordinates including table offsets
                    xc = xc + minTablex + 55
                    yc = yc + minTabley + 25

                    # Convert coordinates to integers
                    xc, yc = int(xc), int(yc)
                    # Get the depth at the calculated coordinates
                    z = get_depth_at(xc, yc)
                    return (1.56 - z) * 1000
                else:
                    pass
    except:
        return None

# Function to detect object coordinates in the image
def coordsX_Y(img):
    # Rotate and enhance the contrast of the image
    img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    img = improve_contrast(img)
    # Convert image to HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    maskT = cv2.erode(img, kernel, iterations=5)
    maskT = cv2.dilate(maskT, kernel, iterations=5)

    # Create a mask by converting to grayscale, thresholding, and applying Canny edge detection
    maskT = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    kernel = np.ones((3, 3), np.uint8)

    maskT = cv2.erode(maskT, kernel, iterations=2)
    maskT = cv2.dilate(maskT, kernel, iterations=2)

    _, maskT = cv2.threshold(maskT, 100, 255, cv2.THRESH_BINARY)

    maskT = cv2.Canny(maskT, 100, 200)

    # Display the processed mask
    cv2.imshow("ImageKinect: localization", maskT)
    cv2.waitKey(3)

    height, width = img.shape[:2]

    # Find contours in the mask
    contours, _ = cv2.findContours(maskT, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if 150 < area < 900:
            # Get the minimum area rectangle for the contour
            rect = cv2.minAreaRect(contour)

            (x,y), (w,h), angle = rect

            if angle > 45 and angle < 90:
                angle = angle - 90

            if area > 470:
                angle = angle + 90

            # Get the bounding box for the contour
            x, y, w, h = cv2.boundingRect(contour)

            # Calculate the center coordinates of the bounding box
            xc = (w / 2) + x
            yc = (h / 2) + y

            yDobot = height // 2
            xDobot = int(width * 0.7474)

            dis = int(ed(xc, xDobot, yc, yDobot))
            disx = abs(xc - xDobot)
            disY = abs(yc - yDobot)
            z = coordZ(maskT, disx, disY, dis, area)

            # Transform coordinates to real-world values
            realX = 0.8 - (xc * 0.8) / width
            realY = ((yc * 1.5) / height)
            realX = (realX - (((width * 0.2625) * 0.8) / width)) * 1000
            realY = (realY - (((height / 2) * 1.5) / height)) * 1000

    return realX, realY, z, xc, yc, angle * -1

# Main function to initialize the ROS node and process incoming messages
def main():
    rospy.init_node('coords', anonymous=True)
    # Subscribe to the relevant topics
    rospy.Subscriber('/tableColor', Image, image_callbackColor)
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, callback_pointcloud)
    rospy.Subscriber("/table_coordinates", coords, callback_tableCoords)
    # Publisher for object coordinates
    objectCoords_pub = rospy.Publisher('/objectCoords', String, queue_size=10)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        try:
            if cv_imageColor is not None:
                # Process the color image to get object coordinates
                x, y, z, xc, yc, angle = coordsX_Y(cv_imageColor)

                # Detect the color of the object
                color = colorDetection(xc, yc)

                if color == "unknown":
                    color, x, y, z, angle = None, None, None, None, None

                # Publish the detected object information if valid
                if color and x and y and z and angle:
                    objectCoords_pub.publish(f"{color}({x},{y},{z},{angle}")
                    
                rate.sleep()
        except Exception as e:
            pass

if __name__ == '__main__':
    main()
