#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from time import time

pub = 0

def odometry_callback(msg):
    print msg.pose.pose
    x = msg.pose.pose.position.x
    twist = Twist()
    if x < 0.4:
        twist.linear.x = 0.05
        twist.linear.y = 0.0
        twist.linear.z = 0.0
    else:
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('move_around', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    reset_odom = rospy.Publisher('reset', Empty, queue_size=10)
    rate = rospy.Rate(2)

    print "wait for subscribing topics"
    rate.sleep()

    print "Reset odom"
    reset_odom.publish(Empty())

    print "Run"
    rate.sleep()
    rospy.Subscriber("odom", Odometry, odometry_callback)
    rospy.spin()

