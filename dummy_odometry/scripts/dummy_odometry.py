#!/usr/bin/env python3
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

def dummy_odometry_publisher():
    # Create publisher for odometry messages
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    # Initialize the node
    rospy.init_node('dummy_odometry_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    # Create a TF broadcaster for dynamic transforms
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Initial pose parameters
    x = 0.0
    y = 0.0
    theta = 0.0

    # Define speeds
    linear_speed = 0.1   # m/s
    angular_speed = 0.1  # rad/s

    while not rospy.is_shutdown():
        # Time step (approximate for 10Hz)
        dt = 0.1

        # Update pose: simple forward movement with rotation
        x += linear_speed * dt
        theta += angular_speed * dt

        # Create an odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Convert yaw (theta) into quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Optionally set twist values if needed
        odom.twist.twist.linear.x = linear_speed
        odom.twist.twist.angular.z = angular_speed

        # Publish the odometry message
        odom_pub.publish(odom)

        # Broadcast the TF transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]
        tf_broadcaster.sendTransform(t)

        rate.sleep()

if __name__ == '__main__':
    try:
        dummy_odometry_publisher()
    except rospy.ROSInterruptException:
        pass
