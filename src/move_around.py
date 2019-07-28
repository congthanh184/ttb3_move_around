import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def odometry_callback(msg):
	print msg.pose.pose

if __name__ == '__main__':
    rospy.init_node('move_around', anonymous=True)
    rospy.Subscriber("odom", Odometry, odometry_callback)
	rospy.spin()

