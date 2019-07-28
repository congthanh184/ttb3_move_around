#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

pub = nil

def odometry_callback(msg):
    print msg.pose.pose
    x = msg.pose.pose.position.x
    twist = Twist()
    if x < 0.2:
        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
    else:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('move_around', anonymous=True)
    rospy.Subscriber("odom", Odometry, odometry_callback)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.spin()

