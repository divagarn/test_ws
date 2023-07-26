#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

map_image = None
map_resolution = None
map_origin_x = None
map_origin_y = None

def draw_arrow(image, x, y, yaw, length=20, color=(0, 0, 255), thickness=2):
    angle = np.deg2rad(yaw)
    x2 = int(x + length * np.cos(angle))
    y2 = int(y + length * np.sin(angle))
    cv2.arrowedLine(image, (x, y), (x2, y2), color, thickness)

def map_callback(msg):
    global map_image, map_resolution, map_origin_x, map_origin_y

    map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    normalized_map = cv2.normalize(map_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    map_image = cv2.cvtColor(normalized_map, cv2.COLOR_GRAY2BGR)

    map_image[np.where((map_data >= 1) & (map_data <= 100))] = 240
    map_image[np.where(map_data == -1)] = 101

    map_resolution = msg.info.resolution
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y

def odom_callback(msg):
    global map_image, map_resolution, map_origin_x, map_origin_y

    if map_image is not None and map_resolution is not None and map_origin_x is not None and map_origin_y is not None:
        x = int((msg.pose.pose.position.x - map_origin_x) / map_resolution)
        y = int((msg.pose.pose.position.y - map_origin_y) / map_resolution)

        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        draw_arrow(map_image, x, y, np.degrees(yaw))

        cv2.imshow('Occupancy Grid Map', map_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()
    cv2.destroyAllWindows()
