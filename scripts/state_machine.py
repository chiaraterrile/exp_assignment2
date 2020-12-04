#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from tf import transformations
import math

flag_play = 0
message = " "

#si blocca allo stato due e non continua con la state machine

#! /usr/bin/env python
# import ros stuff

# it's not an action server, it's just a node which gives the possibility to set a goal and then it will frive the robot toward this position
# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_sleep_ = Point()
desired_position_sleep_.x = -6
desired_position_sleep_.y = 8
desired_position_sleep_.z = 0

desired_position_normal_ = Point()
desired_position_normal_.x = random.randint(-6,7)
desired_position_normal_.y = random.randint(-8,8)
desired_position_normal_.z = 0
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = 3.0  # In ROS Noetic, it may be necessary to change the sign of this proportional controller
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# publishers
pub = None

# callbacks


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


def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)


def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle


def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if twist_msg.angular.z > ub_a:
            twist_msg.angular.z = ub_a
        elif twist_msg.angular.z < lb_a:
            twist_msg.angular.z = lb_a

    pub.publish(twist_msg)

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)


def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)

    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    change_state(0)

def user_action():
			
			global flag_play 
			global message
                 
			return ('sleep')     

# define state RandomlyGoing
class RandomlyGoing(smach.State):
    def __init__(self):
	
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['randomlygoing_counter_in'],
                             output_keys=['randomlygoing_counter_out'])
        
    def execute(self, userdata):
        
	#x = random.randint(0,10)
	#y =random.randint(0,10)
	#random_coordinates = "x= "+str(x)+" y= "+str(y)
        
        global pub, active_
# in the main you have a kind of finite state machine
        

        pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=1)

        sub_odom = rospy.Subscriber('robot/odom', Odometry, clbk_odom)
        print ('Going to : ', desired_position_normal_)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
                if state_ == 0: #where the robot fix its position in order to reach the target
                        fix_yaw(desired_position_normal_)
                        
                elif state_ == 1: #the robot just go ahead
                        go_straight_ahead(desired_position_normal_)
                elif state_ == 2: #final state, once the goal is reached
                        
                        done()
                        print('reached position')
                        time.sleep(5)
                        return user_action()
                else:
                        rospy.logerr('Unknown state!')

                rate.sleep()
   	
        

        rospy.loginfo('Executing state RANDOMLYGOING (users = %f)'%userdata.randomlygoing_counter_in)
        userdata.randomlygoing_counter_out = userdata.randomlygoing_counter_in + 1
	
        
    

# define state Sleeping
class Sleeping(smach.State):
    def __init__(self):
	

        smach.State.__init__(self, 
			     outcomes=['normal'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])
        
    def execute(self, userdata):
        
        #state_ = 0
    
        global pub, active_
        
# in the main you have a kind of finite state machine
        

        pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=1)

        sub_odom = rospy.Subscriber('robot/odom', Odometry, clbk_odom)
        print ('Going to sleep ')

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
                if state_ == 0: #where the robot fix its position in order to reach the target
                        fix_yaw(desired_position_sleep_)
                        
                elif state_ == 1: #the robot just go ahead
                        go_straight_ahead(desired_position_sleep_)
                elif state_ == 2: #final state, once the goal is reached
                        
                        done()
                        print('reached home')
                        time.sleep(10)
                        return ('normal')
                else:
                        rospy.logerr('Unknown state!')

                rate.sleep() 
        
   	
	#remains in the sleeping state for a certain amount of time
        

        rospy.loginfo('Executing state SLEEPING (users = %f)'%userdata.sleeping_counter_in)
        userdata.sleeping_counter_out = userdata.sleeping_counter_in + 1
	
        #after being in the sleeping state it comes back to normal
        

# define state Playing
class Playing(smach.State):
    def __init__(self):
	global flag_play
	global message
	global person
        smach.State.__init__(self, 
                             outcomes=['normal'],
                             input_keys=['playing_counter_in'],
                             output_keys=['playing_counter_out'])
       

    def execute(self, userdata):
       
        #defining the publisher
   	#pub = rospy.Publisher('target', String, queue_size=10)
	#target = person
   	#pub.publish(target)
	#time.sleep(5)
    	#target = message
   	#pub.publish(target)
	#time.sleep(5)
	#target = person
   	#pub.publish(target)

        rospy.loginfo('Executing state PLAYING (users = %f)'%userdata.playing_counter_in)
        userdata.playing_counter_out = userdata.playing_counter_in + 1
	time.sleep(10)
	#after being in the playing state it comes back to normal
        return ('normal')
        


        
def main():
  
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('RANDOMLYGOING', RandomlyGoing(), 
                               transitions={'normal':'RANDOMLYGOING', 
                                            'sleep':'SLEEPING',
					    'play':'PLAYING'},
                               remapping={'randomlygoing_counter_in':'sm_counter', 
                                          'randomlygoing_counter_out':'sm_counter'})
       
        smach.StateMachine.add('SLEEPING', Sleeping(), 
                               transitions={'normal':'RANDOMLYGOING' 
                                            
					    },
                               remapping={'sleeping_counter_in':'sm_counter',
                                          'sleeping_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAYING', Playing(), 
                               transitions={'normal':'RANDOMLYGOING'}, 
                                            
							
                               remapping={'playing_counter_in':'sm_counter',
                                          'plying_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    
    
    

if __name__ == '__main__':
	#state_machine()
        main()

