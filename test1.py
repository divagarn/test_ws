#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import heapq

map_image = None
map_resolution = None
map_origin_x = None
map_origin_y = None

rectangle_size = 10

selected_point = None
path_points = []

# Define a class for A* node
class AStarNode:
    def __init__(self, x, y, parent=None, g_cost=0, h_cost=0):
        self.x = x
        self.y = y
        self.parent = parent
        self.g_cost = g_cost
        self.h_cost = h_cost

    def f_cost(self):
        return self.g_cost + self.h_cost

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

def draw_path(image, points, color=(0, 255, 0), thickness=2):
    for i in range(len(points) - 1):
        cv2.line(image, points[i], points[i + 1], color, thickness)

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

def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def calculate_point_in_front(x, y, yaw, distance=0.5):
    angle = np.deg2rad(yaw)
    x_in_front = x + distance * np.cos(angle)
    y_in_front = y + distance * np.sin(angle)
    return x_in_front, y_in_front

def a_star_pathfinding(start, goal, map_data):
    open_set = []
    closed_set = set()

    heapq.heappush(open_set, (start.f_cost(), start))

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current.x == goal.x and current.y == goal.y:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]

        closed_set.add((current.x, current.y))

        for dx in range(-1, 2):
            for dy in range(-1, 2):
                if dx == 0 and dy == 0:
                    continue

                next_x, next_y = current.x + dx, current.y + dy
                if 0 <= next_x < map_data.shape[1] and 0 <= next_y < map_data.shape[0]:
                    if map_data[next_y, next_x] != 255 and (next_x, next_y) not in closed_set:
                        g_cost = current.g_cost + calculate_distance(current.x, current.y, next_x, next_y)
                        h_cost = calculate_distance(next_x, next_y, goal.x, goal.y)
                        new_node = AStarNode(next_x, next_y, current, g_cost, h_cost)
                        if (next_x, next_y) not in [item[1].x for item in open_set]:
                            heapq.heappush(open_set, (new_node.f_cost(), new_node))

    return None

def odom_callback(msg):
    global map_image, map_resolution, map_origin_x, map_origin_y, rectangle_size, selected_point, path_points

    if map_image is not None and map_resolution is not None and map_origin_x is not None and map_origin_y is not None:
        x = int((msg.pose.pose.position.x - map_origin_x) / map_resolution)
        y = int((msg.pose.pose.position.y - map_origin_y) / map_resolution)

        orientation_q = msg.pose.pose.orientation
        x_robot, y_robot, yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

        # Calculate the point 0.5 meters in front of the robot
        x_in_front, y_in_front = calculate_point_in_front(x, y, np.degrees(yaw), distance=0.5)

        # Mark the selected point as red on the map
        selected_point = (int(x_in_front), int(y_in_front))
        map_image[selected_point[1], selected_point[0]] = (0, 0, 255)

        thresh = 50
        y_crop_min = max(0, y - thresh)
        y_crop_max = min(map_image.shape[0], y + thresh)
        x_crop_min = max(0, x - thresh)
        x_crop_max = min(map_image.shape[1], x + thresh)

        cropped_map_image = map_image[y_crop_min:y_crop_max, x_crop_min:x_crop_max]

        draw_rectangle(map_image, x, y, rectangle_size, np.degrees(yaw))
        draw_arrow(map_image, x, y, np.degrees(yaw))

        if selected_point:
            # Convert map_image to map_data (grayscale) for A* pathfinding
            map_data = cv2.cvtColor(map_image, cv2.COLOR_BGR2GRAY)

            start_node = AStarNode(x, y)
            goal_node = AStarNode(selected_point[0], selected_point[1])

            # Find path using A* algorithm
            path_points = a_star_pathfinding(start_node, goal_node, map_data)

            # Draw the path on the map in green color
            draw_path(map_image, path_points)

        cv2.imshow('OccupancyMap', cv2.flip(map_image, 0))
        cv2.imshow('Occupancy Grid Map', cv2.resize(cv2.flip(cropped_map_image, 0), None, fx=4, fy=4))
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()
    cv2.destroyAllWindows()
