#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import cv2
import numpy as np
import os

cv_image = None
points = None
bridge = CvBridge()

# Load the reference image outside the Dcoords function
image_route = str(os.path.dirname(__file__)) + "/TableEmpty.png"
image1 = cv2.imread(image_route)

def image_callback(msg):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(e)
        return
def callback_pointcloud(msg):
    global points
    try:
        points = msg
    except:
        pass

def coordsX_Y(img):
    dim = (448, 336) #30% resize - original 640x480
    resize = cv2.resize(img, dim)
    table_crop = resize[30:235, 30:416]

    # Resize image1 to match the size of table_crop
    image1_resized = cv2.resize(image1, (table_crop.shape[1], table_crop.shape[0]))

    # Compute the absolute difference between the images
    diff = cv2.absdiff(image1_resized, table_crop)

    # Convert the difference image to grayscale
    diff = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)

    # Find contours in the thresholded image
    contours, _ = cv2.findContours(diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours on the original images
    cv2.drawContours(table_crop, contours, -1, (0, 0, 0), 2)
    
    x, y, w, h = 0, 0, 0, 0

    for contour in contours:
    # Approximate the contour to a polygon
        epsilon = 0.05 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Check if the polygon has 4 corners (potentially a square or rectangle)
        if len(approx) == 4:
            # Check if the contour area is large enough to be considered a cube
            if cv2.contourArea(contour) > 10:
                # Get the bounding box coordinates of the first contour
                x, y, w, h = cv2.boundingRect(contour)
    xc = (w/2) + x
    yc = (h/2) + y

    realX = (x * 1.5) / 386
    realY = (y * 0.8) / 205

    # Print the coordinates
    #print("\nxImagen: ", xc)
    #print("yImagen: ", yc)
    #print("xReal: ", realX)
    #print("yReal: ", realY)

    return table_crop, xc, yc, realX, realY

def coordZ(xT, yT):
    target_xPx = (xT * 448) / 640
    target_yPx = (yT * 336) / 480

    target_x = (target_xPx * 1.5) / 612
    target_y = (target_yPx * 0.8) / 331

    #print("target_xPx", target_xPx)
    #print("target_yPx", target_yPx)
    #print("target_x", target_x)
    #print("target_y", target_y)
    
    nearest_point = None
    min_distance = float('inf')

    for point in pc2.read_points(points, field_names=("x", "y", "z"), skip_nans=True):
        # Extract the x, y, z coordinates of the point
        x, y, z = point
        
        # Calculate the Euclidean distance between the point and the target coordinates
        distance = np.sqrt((target_x - x)**2 + (target_y - y)**2)
        
        # Update nearest point if the distance is smaller than the current minimum
        if distance < min_distance:
            nearest_point = point
            min_distance = distance
    
    if nearest_point is not None:
        _, _, z = nearest_point
        return z
    else:
        return None, None, None

def main():
    rospy.init_node('coord', anonymous=True)
    rospy.Subscriber('/depth_image', Image, image_callback)
    rospy.Subscriber("/cameraTecho/depth/points", PointCloud2, callback_pointcloud)

    while not rospy.is_shutdown():
        if cv_image is not None:  # Check if cv_image is not None
            try:
                image, xPx, yPx, x, y = coordsX_Y(cv_image)
                z = (1.4 - 0.03) - coordZ(xPx, yPx)
                y = 0.8 - y
                print("x, y, z", x, y, z)
                cv2.imshow("Image", image)
                cv2.waitKey(3)
                #break
            except KeyboardInterrupt:
                print("Shutting down...")
                cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
