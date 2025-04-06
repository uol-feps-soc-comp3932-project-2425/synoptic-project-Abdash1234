#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    rospy.loginfo("Map size: %d x %d", msg.info.width, msg.info.height)
    rospy.loginfo("Map resolution: %f", msg.info.resolution)
    rospy.loginfo("Map origin: [%.2f, %.2f, %.2f]",
                  msg.info.origin.position.x,
                  msg.info.origin.position.y,
                  msg.info.origin.position.z)

def main():
    rospy.init_node('map_listener')
    rospy.Subscriber("/map", OccupancyGrid, map_callback)
    rospy.loginfo("Map listener node started...")
    rospy.spin()

if __name__ == '__main__':
    main()
