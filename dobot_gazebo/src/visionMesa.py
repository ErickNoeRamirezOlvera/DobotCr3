#!/usr/bin/env python3
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
cv_image=0

def callback(data):
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':
    image_pub = rospy.Publisher("color_deseado",String,queue_size=10)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/cameraTecho/depth/image_raw",Image,callback)
    rospy.init_node('image_converter', anonymous=True)
    
    while not rospy.is_shutdown():
        dim=(720,504)
        cv_image=cv2.resize(cv_image, dim)
        image = cv_image

        original = cv_image
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([110,50,50]) 
        upper_blue = np.array([130,255,255]) 

        mask = cv2.inRange(image, lower_blue, upper_blue) 

        #rospy.loginfo(mask)
        sum_horizontal = np.sum(mask, axis = 0)
        sum_vertical = np.sum(mask, axis = 1)

        rospy.loginfo(mask)

        #Calculate the borders of the table
        horizontal_primer = res = next(x for x, val in enumerate(sum_horizontal)
                                    if val > 100)

        vertical_primer = res = next(x for x, val in enumerate(sum_vertical)
                                    if val > 100)
        
        horizontal_ultimo_val = res = np.where(sum_horizontal > 100)[0]
        vertical_ultimo_val = res = np.where(sum_vertical > 100)[0]

        horizontal_ultimo = horizontal_ultimo_val[-1]
        vertical_ultimo = vertical_ultimo_val[-1]

        rospy.loginfo(horizontal_primer)
        rospy.loginfo(vertical_primer)
        rospy.loginfo(horizontal_ultimo)
        rospy.loginfo(vertical_ultimo)

        #Draw the border of the table
        #Cirle 1
        center_coordinates = (horizontal_primer, vertical_primer)
        radius = 10
        color = (0, 255, 0)
        thickness = 2
        image_with_circle1 = cv2.circle(original, center_coordinates, radius, color, thickness)

        #Cirle 2
        center_coordinates = (horizontal_primer, vertical_ultimo)
        image_with_circle2 = cv2.circle(image_with_circle1, center_coordinates, radius, color, thickness)

        #Cirle 3
        center_coordinates = (horizontal_ultimo, vertical_primer)
        image_with_circle3 = cv2.circle(image_with_circle2, center_coordinates, radius, color, thickness)

        #Cirle 4
        center_coordinates = (horizontal_ultimo, vertical_ultimo)
        image_with_circle4 = cv2.circle(image_with_circle3, center_coordinates, radius, color, thickness)

        color = "Rojo"

        #cv2.imshow("Image window", image)

        cv2.imshow('mask', mask)
        cv2.imshow('original', original)
        cv2.imshow("Esquinas", image_with_circle4)
        cv2.waitKey(3)

        try:
            image_pub.publish(color)

        except CvBridgeError as e:
            print(e)