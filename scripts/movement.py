#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import math

x = 0.0
y = 0.0
theta = 0.0

def newOdom (msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber ("robot/odom",Odometry,newOdom)
pub = rospy.Publisher ("robot/cmd_vel",Twist,queue_size = 1)

speed = Twist()

r = rospy.Rate(4)

goal = Point()
goal.x = -3
goal.y = 3
#inc_x = goal.x - x 
#inc_y = goal.y - y 

#angle_to_goal = atan2(inc_y,inc_x)


inc_x = goal.x- x 
inc_y = goal.y - y

angle_to_goal = atan2(inc_y,inc_x)
dist_to_goal = math.sqrt(pow((goal.x-x),2)+pow((goal.x-x),2))

#while not rospy.is_shutdown():
while dist_to_goal > 0.1 :
    inc_x = goal.x- x 
    inc_y = goal.y - y

    angle_to_goal = atan2(inc_y,inc_x)
    dist_to_goal = math.sqrt(pow((goal.x-x),2)+pow((goal.x-x),2))
    print(dist_to_goal)
    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 1
    else :
        speed.linear.x = 0.5
        speed.angular.z = 0.0
        #angle_to_goal = theta 
    
            
    
    pub.publish(speed)
    r.sleep()

speed.linear.x = 0.0
speed.angular.z = 0.0
pub.publish(speed)         
#speed.linear.x = 0.0
#speed.angular.z = 0.0
#pub.publish(speed)

   
    