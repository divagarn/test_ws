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

rectangle_size = 10

def draw_arrow(image, x, y, yaw, length=20, color=(255, 0, 0), thickness=1):
    angle = np.deg2rad(yaw)
    x2 = int(x + length * np.cos(angle))
    y2 = int(y + length * np.sin(angle))
    cv2.arrowedLine(image, (x, y), (x2, y2), color, thickness)

def draw_rectangle(image, x, y, size, yaw, color=(0, 0, 255), thickness=2):
    # Calculate the points of the rotated rectangle
    angle_rad = np.deg2rad(yaw)
    h, w = size, size
    cos_val = np.cos(angle_rad)
    sin_val = np.sin(angle_rad)
    x0 = int(x + w/2 * cos_val - h/2 * sin_val)
    y0 = int(y + w/2 * sin_val + h/2 * cos_val)
    x1 = int(x + w/2 * cos_val + h/2 * sin_val)
    y1 = int(y + w/2 * sin_val - h/2 * cos_val)
    x2 = int(x - w/2 * cos_val + h/2 * sin_val)
    y2 = int(y - w/2 * sin_val - h/2 * cos_val)
    x3 = int(x - w/2 * cos_val - h/2 * sin_val)
    y3 = int(y - w/2 * sin_val + h/2 * cos_val)
    
    # Draw the rotated rectangle
    cv2.line(image, (x0, y0), (x1, y1), color, thickness)
    cv2.line(image, (x1, y1), (x2, y2), color, thickness)
    cv2.line(image, (x2, y2), (x3, y3), color, thickness)
    cv2.line(image, (x3, y3), (x0, y0), color, thickness)

def map_callback(msg):
    global map_image, map_resolution, map_origin_x, map_origin_y

    map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    normalized_map = cv2.normalize(map_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    map_image = cv2.cvtColor(normalized_map, cv2.COLOR_GRAY2BGR)

    map_image[np.where((map_data >= 1) & (map_data <= 100))] = (0, 255, 0)  
    map_image[np.where(map_data == -1)] = (101, 101, 101)  

    map_resolution = msg.info.resolution
    map_origin_x = msg.info.origin.position.x
    map_origin_y = msg.info.origin.position.y

def odom_callback(msg):
    global map_image, map_resolution, map_origin_x, map_origin_y, rectangle_size

    if map_image is not None and map_resolution is not None and map_origin_x is not None and map_origin_y is not None:
        x = int((msg.pose.pose.position.x - map_origin_x) / map_resolution)
        y = int((msg.pose.pose.position.y - map_origin_y) / map_resolution)

        orientation_q = msg.pose.pose.orientation
        x_robot, y_robot, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        print("x", x_robot, "y", y_robot, "yaw", yaw)

        thresh = 50
        y_crop_minuz = (y - thresh)
        y_crop_add = (y + thresh)

        x_crop_minuz = (x - thresh)
        x_crop_add = (x + thresh)

        cropped_map_image = map_image[y_crop_minuz:y_crop_add, x_crop_minuz:x_crop_add]

        draw_rectangle(map_image, x, y, rectangle_size, np.degrees(yaw))
        draw_arrow(map_image, x, y, np.degrees(yaw))
        cv2.imshow('OccupancyMap', cv2.flip(map_image, 0))

        cv2.imshow('Occupancy Grid Map', cv2.resize(cv2.flip(cropped_map_image, 0), None, fx=4, fy=4))
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()
    cv2.destroyAllWindows()
