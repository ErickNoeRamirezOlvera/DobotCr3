#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np

bridge = CvBridge()

def callback_pointcloud(data):
    try:
        pc = np.array(list(pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))))

        # Extract depth information
        depth = pc[:, 2]

        # Normalize depth values to uint8 range (0-255)
        min_depth = np.min(depth)
        max_depth = np.max(depth)
        depth_normalized = ((depth - min_depth) / (max_depth - min_depth) * 255).astype(np.uint8)

        # Reshape depth data into an image format
        height, width = data.height, data.width
        depth_image = depth_normalized.reshape((height, width))

        # Convert depth image to ROS Image message
        depth_image_msg = bridge.cv2_to_imgmsg(depth_image, encoding="mono8")

        # Publish depth image
        depth_image_pub.publish(depth_image_msg)

    except:
        pass

def main():
    rospy.init_node('pointcloud_to_image', anonymous=True)

    # Subscribe to PointCloud2 topic
    rospy.Subscriber("/cameraTecho/depth/points", PointCloud2, callback_pointcloud)

    # Initialize depth image publisher
    global depth_image_pub
    depth_image_pub = rospy.Publisher("/depth_image", Image, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()

