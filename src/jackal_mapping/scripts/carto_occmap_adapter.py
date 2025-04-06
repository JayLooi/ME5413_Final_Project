#!/usr/bin/python3
import rospy
from nav_msgs.msg import OccupancyGrid


OBSTACLE_THRES = 75
UNKNOWN = 50
FREE = 0


def callback(cmap: OccupancyGrid):
    data = list(cmap.data)
    for y in range(cmap.info.height):
        for x in range(cmap.info.width):
            i = x + (cmap.info.height - 1 - y) * cmap.info.width
            if data[i] >= OBSTACLE_THRES:  
                data[i] = 100
            elif (data[i] >= FREE) and (data[i] < UNKNOWN):
                data[i] = 0
            else:
                data[i] = -1
    cmap.data = tuple(data)
    pub.publish(cmap)


rospy.init_node('mapc_node', anonymous=True)
sub = rospy.Subscriber('/map', OccupancyGrid, callback)
pub = rospy.Publisher('/explore_lite_map', OccupancyGrid, queue_size=20)

rospy.spin()
