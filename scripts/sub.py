#! /usr/bin/env python
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
# it's not an action server, it's just a node which gives the possibility to set a goal and then it will frive the robot toward this position
# robot state variables

def clbk_odom(msg):
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def main():
    global pub, active_
# in the main you have a kind of finite state machine
    
    rospy.init_node('subscriber', anonymous=True)
    while not rospy.is_shutdown():
        sub_odom = rospy.Subscriber('robot/odom', Odometry, clbk_odom)
    
    


if __name__ == '__main__':
    main()
