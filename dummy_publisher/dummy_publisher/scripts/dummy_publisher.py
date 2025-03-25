#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    # This publisher mimics sensor data from the e-puck2
    pub = rospy.Publisher('dummy_topic', String, queue_size=10)
    rospy.init_node('dummy_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz publishing rate
    while not rospy.is_shutdown():
        message = "Simulated e-puck data"
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
