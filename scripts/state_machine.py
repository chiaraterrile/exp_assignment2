#!/usr/bin/env python

import roslib
import time
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
import numpy as np
from scipy.ndimage import filters
import imutils
import cv2
from sensor_msgs.msg import CompressedImage




####################################### for the navigation #######################################

# it's not an action server, it's just a node which gives the possibility to set a goal and then it will frive the robot toward this position
# robot state variables
detected_object = 0
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
#desired_position_normal_.x = random.randint(-6,7)
#desired_position_normal_.y = random.randint(-8,8)
desired_position_normal_.x = -5
desired_position_normal_.y = 6
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
#counter_time = 0

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
####################################### for tracking the ball #######################################
VERBOSE = False
def call_play(data):
   	 	
            global detected_object
            global message
            message = data.data
            #checking that there's actually a message from the publisher
            if message == "1" :
                detected_object = 1
                
            else :
                detected_object = 0
            #print('end_proc = ' ,detected_object)

class image_feature:
    global detected_object,detected_object #counter_time
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
       
     # topic where we publish
        self.image_pub = rospy.Publisher("robot/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("robot/cmd_vel",
                                       Twist, queue_size=1)
        #self.pub_play = rospy.Publisher('object', String, queue_size=10) 
        # subscribed Topic
        self.subscriber = rospy.Subscriber("robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
        
        self.pub_play = rospy.Publisher('object', String, queue_size=10) 
        
    def callback(self, ros_data):
        #cv2.destroyAllWindows()
        detected_object = 0
        #counter_timer = 0
        if VERBOSE:
                print ('received image of type: "%s"' % ros_data.format)
            
            
            #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        greenLower = (50, 50, 20) #used to define the color we want to find
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower,greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
            # only proceed if at least one contour was found
        
        if len(cnts) > 0:
                
                msg = "1"
                self.pub_play.publish(msg)
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                            (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    
                    vel.angular.z = -0.002*(center[0]-400) #inversely prop to the distance from the center of the actual image and the center of the detected image
                    vel.linear.x = -0.01*(radius-100) #inv prop to the difference btw the radious of the detected object and a fixed radious set to 100
                    # if radious become smaller, also the detected obj is smaller -> far from the desired position -> linear velocity will become positive
                    self.vel_pub.publish(vel)
                    

                else: #if the radious is smaller than 10, I will give you a linear velocity to get close to the object
                    vel = Twist()
                    
                    vel.linear.x = 0.5
                    self.vel_pub.publish(vel)
                #cv2.imshow('window', image_np)
        else: # if I'm not detecting obj, I rotate in order to find it
                #cv2.imshow('window', image_np)
                #pub_play = rospy.Publisher('object', String, queue_size=10) 
                msg = "0"
                self.pub_play.publish(msg)
                vel = Twist()
                #rate = rospy.Rate(20)
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                
        cv2.imshow('window', image_np)
            
        cv2.waitKey(2)
        #cv2.destroyAllWindows()
            
       
        
####################################### state machine functions ##########################

def user_action():
    global detected_object
    global message
    return ('play')

# define state RandomlyGoing
class RandomlyGoing(smach.State):
    def __init__(self):
	
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['randomlygoing_counter_in'],
                             output_keys=['randomlygoing_counter_out'])
        
    def execute(self, userdata):
    
        
        global pub, active_,detected_object
# in the main you have a kind of finite state machine
        detected_object = 0
        
        
        pub = rospy.Publisher('robot/cmd_vel', Twist, queue_size=1)

        sub_odom = rospy.Subscriber('robot/odom', Odometry, clbk_odom)
        print ('Going to : ', desired_position_normal_)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
                #cv2.destroyAllWindows()  
                if state_ == 0: #where the robot fix its position in order to reach the target
                        fix_yaw(desired_position_normal_)
                        
                elif state_ == 1: #the robot just go ahead
                        go_straight_ahead(desired_position_normal_)
                elif state_ == 2: #final state, once the goal is reached
                        
                        done()
                        print('reached position')
                        time.sleep(5)
                        change_state(0)
                        #cv2.destroyAllWindows()  
                        return ('sleep')
                else:
                        rospy.logerr('Unknown state!')

                rate.sleep()
   	
        

        rospy.loginfo('Executing state RANDOMLYGOING (users = %f)'%userdata.randomlygoing_counter_in)
        userdata.randomlygoing_counter_out = userdata.randomlygoing_counter_in + 1
	
        
    

# define state Sleeping
class Sleeping(smach.State):
    def __init__(self):
	

        smach.State.__init__(self, 
			     outcomes=['play'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])
        
    def execute(self, userdata):
        
        #state_ = 0
    
        global pub, active_,detected_object #counter_time
        
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
                            time.sleep(5)
                            cv2.destroyAllWindows()
                            return ('play')
            else:
                            rospy.logerr('Unknown state!')

            rate.sleep() 
        
   	
	#remains in the sleeping state for a certain amount of time
        
        #cv2.destroyAllWindows()
        rospy.loginfo('Executing state SLEEPING (users = %f)'%userdata.sleeping_counter_in)
        userdata.sleeping_counter_out = userdata.sleeping_counter_in + 1
	
        #after being in the sleeping state it comes back to normal
        

# define state Playing
class Playing(smach.State):
    def __init__(self):
	
	global message,detected_object
    
        smach.State.__init__(self, 
                             outcomes=['normal'],
                             input_keys=['playing_counter_in'],
                             output_keys=['playing_counter_out'])
       

    def execute(self, userdata):
        global detected_object
        #cv2.destroyAllWindows() 
        
        print(detected_object)
        rate = rospy.Rate(20)
        #while not rospy.is_shutdown():
        im = image_feature()
        print('inizio tempo')
        t_end = time.time() + 60 # 60
        while time.time() < t_end :

                        #im = image_feature()
                        rospy.Subscriber('object', String, call_play)
                        print(detected_object)
                            
                        if detected_object == 1 :
                                print('inizio tempo')
                                t_end = time.time() +60 # 60
                                rospy.Subscriber('object', String, call_play)
                            
                        #rate.sleep() 
                           
        cv2.destroyAllWindows()               
        print('no object found')
        change_state(0)
        return ('normal')
        #im = image_feature()     

        
        rospy.loginfo('Executing state PLAYING (users = %f)'%userdata.playing_counter_in)
        userdata.playing_counter_out = userdata.playing_counter_in + 1
	
        


        
def main():
  
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
       
        smach.StateMachine.add('PLAYING', Playing(), 
                               transitions={'normal':'RANDOMLYGOING'
					    },
                                            
							
                               remapping={'playing_counter_in':'sm_counter',
                                          'plying_counter_out':'sm_counter'})

        smach.StateMachine.add('RANDOMLYGOING', RandomlyGoing(), 
                               transitions={'normal':'RANDOMLYGOING', 
                                            'sleep':'SLEEPING',
					    'play':'PLAYING'},
                               remapping={'randomlygoing_counter_in':'sm_counter', 
                                          'randomlygoing_counter_out':'sm_counter'})
       
        smach.StateMachine.add('SLEEPING', Sleeping(), 
                               transitions={'play':'PLAYING'
					    },
                               remapping={'sleeping_counter_in':'sm_counter',
                                          'sleeping_counter_out':'sm_counter'})

        

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

