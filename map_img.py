#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    normalized_map = cv2.normalize(map_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    map_image = cv2.cvtColor(normalized_map, cv2.COLOR_GRAY2BGR)

    map_image[np.where((map_data >= 1) & (map_data <= 100))] = 240
    map_image[np.where(map_data == -1)] = 101
    cv2.imshow('Occupancy Grid Map', map_image)
    cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()
    cv2.destroyAllWindows()
