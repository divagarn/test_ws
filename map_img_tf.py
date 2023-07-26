#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import tf
from nav_msgs.msg import OccupancyGrid

def get_map_in_robot_frame(map_data, map_info):
    try:
        listener = tf.TransformListener()
        (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Error getting transformation between '/map' and '/base_link'")
        return None

    # Use the transformation to convert the map to the robot's frame
    map_image = np.array(map_data, dtype=np.int8).reshape((map_info.height, map_info.width))
    normalized_map = cv2.normalize(map_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    map_in_robot_frame = cv2.warpAffine(normalized_map, np.array([[rot[1], rot[0], trans[0]], [-rot[0], rot[1], trans[1]]]), (map_info.width, map_info.height))
    return map_in_robot_frame

def map_callback(msg):
    map_image = get_map_in_robot_frame(msg.data, msg.info)
    if map_image is not None:
        map_image[np.where((map_image >= 1) & (map_image <= 100))] = 240
        map_image[np.where(map_image == -1)] = 101

        cv2.imshow('Occupancy Grid Map in Robot Frame', map_image)
        cv2.waitKey(1)

if __name__ == "__main__":
    rospy.init_node('map_subscriber', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.spin()
    cv2.destroyAllWindows()
