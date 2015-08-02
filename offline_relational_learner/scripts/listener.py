#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import GridCells, OccupancyGrid

def callback_map(data):
    print "\nmap_callback:"
    rospy.loginfo(data.header)

def callback_grid(data):
    print "\ngrid_callback:"
    rospy.loginfo(data.header)

def callback_occu(data):
    print "\noccu_callback:"
    rospy.loginfo(data)

def grid_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/trajectory_behaviours/grid", GridCells, callback_grid)

def occu_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/trajectory_behaviours/occu", OccupancyGrid, callback_occu)

def map_listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/map", OccupancyGrid, callback_map)

if __name__ == '__main__':
    #map_listener()
    #occu_listener()
    grid_listener()
    rospy.spin()
