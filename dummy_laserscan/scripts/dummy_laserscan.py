#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

def dummy_laserscan_publisher():
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.init_node('dummy_laserscan_publisher', anonymous=True)
    rate = rospy.Rate(10)  # Publish at 10 Hz
    
    scan = LaserScan()
    scan.header.frame_id = "laser_frame"
    scan.angle_min = -math.pi / 2
    scan.angle_max = math.pi / 2
    scan.angle_increment = math.pi / 180  # 1° increments
    scan.time_increment = 0.0
    scan.scan_time = 0.1
    scan.range_min = 0.2
    scan.range_max = 10.0

    num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
    # For simplicity, simulate all ranges as 5.0 meters.
    scan.ranges = [5.0] * num_readings
    scan.intensities = [0.0] * num_readings

    while not rospy.is_shutdown():
        scan.header.stamp = rospy.Time.now()
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        dummy_laserscan_publisher()
    except rospy.ROSInterruptException:
        pass
