#!/usr/bin/env python

"""!
@author Terrile Chiara
@mainpage Assignment 2 ExpRob
@package exp_assignment2
@section Description
This scripts is a ROS node that implements a FSM that according to what detects the camera switch to one state or another
"""

# Imports
import roslib
import time
import rospy
import smach
import smach_ros
import time
import random
from std_msgs.msg import String,Float64
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
import actionlib
from exp_assignment2.msg import *
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry


# Global variables

## Coordinates of the home position in the sleep state
desired_position_sleep_ = Point()
desired_position_sleep_.x = -6
desired_position_sleep_.y = 8
desired_position_sleep_.z = 0

## Coordinates of the position in the normal state
desired_position_normal_ = Point()

## flag that indicates wheter there's an object or not 
detected_object = 0

# Functions for tracking the ball

VERBOSE = False


def call_play(data):
            """! This function is a callback that if receives '1' as message puts the flag detected_object equal to 1 """
            global detected_object
            
            message = data.data
            
            if message == "1" :
                detected_object = 1
                
            else :
                detected_object = 0
           

class image_feature:
    """! define the class for tracking the ball"""
    global detected_object
    def __init__(self):
        """! Initialize publisher and subscriber"""
       
     
        self.image_pub = rospy.Publisher("robot/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("robot/cmd_vel",
                                       Twist, queue_size=1)
        
        self.subscriber = rospy.Subscriber("robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
        
        ##topic to publish a message in case of detection (or not) of the ball
        self.pub_play = rospy.Publisher('object', String, queue_size=10) 
        
    def callback(self, ros_data):
        
        detected_object = 0
        
        if VERBOSE:
                print ('received image of type: "%s"' % ros_data.format)
            
            
            #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        ## used to define the green color of the ball
        greenLower = (50, 50, 20) 
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower,greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
            
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
            
        

        if len(cnts) > 0:
                # if the object is detected the pub:play publishes a message equal to 1
                msg = "1"
                self.pub_play.publish(msg)
               
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
                    
                    vel.angular.z = -0.002*(center[0]-400) 
                    vel.linear.x = -0.01*(radius-100) 
                    
                    self.vel_pub.publish(vel)
                    

                else: #if the radious is smaller than 10, I will give you a linear velocity to get close to the object
                    vel = Twist()
                    
                    vel.linear.x = 0.5
                    self.vel_pub.publish(vel)
                
        else: # if I'm not detecting obj, I rotate in order to find it
                #if isn't detected any topic pub_play publishes a message equal to 0
                msg = "0"
                self.pub_play.publish(msg)

                vel = Twist()
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                
        cv2.imshow('window', image_np)
            
        cv2.waitKey(2)
        
       
# FMS functions
#def user_action():

    #global detected_object
    #global message
    #return ('play')

# define state RandomlyGoing
class RandomlyGoing(smach.State):
    """! Define the RandomlyGoing state (normal) """
    def __init__(self):
	
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['randomlygoing_counter_in'],
                             output_keys=['randomlygoing_counter_out'])
        
    def execute(self, userdata):
    
        """! Normal state execution 
        @section Description
        In this state is generated every time a new Point desired_position_normal_ in a radom way
        This goal position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the sleep state
        """
        
        global pub, active_,detected_object,desired_position_normal_
# in the main you have a kind of finite state machine
        
        desired_position_normal_.x = random.randint(-6,7)
        desired_position_normal_.y = random.randint(-8,8)
        desired_position_normal_.z = 0

        print('I am moving to: ', desired_position_normal_)

        ## definition of the action client
        client = actionlib.SimpleActionClient('/robot/reaching_goal', exp_assignment2.msg.PlanningAction)
        client.wait_for_server()

        # assigning to the goal position the desired_position_normal coordinates      
        goal = exp_assignment2.msg.PlanningGoal()
        goal.target_pose.pose.position.x = desired_position_normal_.x
        goal.target_pose.pose.position.y = desired_position_normal_.y
        #sending the goal to the server
        client.send_goal(goal)
       
        client.wait_for_result()
                
        print('reached position')
        time.sleep(5)
        return ('sleep')
        rospy.loginfo('Executing state RANDOMLYGOING (users = %f)'%userdata.randomlygoing_counter_in)
        userdata.randomlygoing_counter_out = userdata.randomlygoing_counter_in + 1
	
        
    

# define state Sleeping
class Sleeping(smach.State):
    def __init__(self):
	"""! Define the Sleeping state  """

        smach.State.__init__(self, 
			     outcomes=['play'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])
        
    def execute(self, userdata):
        
        """! Sleeping state execution 
        @section Description
        In this state the home position is sent trough an action client to the server that makes the robot move toward the goal position
        @return the play state
        """
    
        global pub, active_,detected_object,desired_position_sleep_ 
        
        print('I am moving to home : ', desired_position_sleep_)
        ## defintion of the action client
        client = actionlib.SimpleActionClient('/robot/reaching_goal', exp_assignment2.msg.PlanningAction)

        client.wait_for_server()

        # assigning to the goal position the desired_position_sleep coordinates       
        goal = exp_assignment2.msg.PlanningGoal()
        goal.target_pose.pose.position.x = desired_position_sleep_.x
        goal.target_pose.pose.position.y = desired_position_sleep_.y

        # sending the goal to the server 
        client.send_goal(goal)

                
        client.wait_for_result()
        print('reached position')

        time.sleep(10)
        
        return ('play')
   	
	
        rospy.loginfo('Executing state SLEEPING (users = %f)'%userdata.sleeping_counter_in)
        userdata.sleeping_counter_out = userdata.sleeping_counter_in + 1
	
        
        

# define state Playing
class Playing(smach.State):
    def __init__(self):
	"""! Define the Playing state  """
	
    
        smach.State.__init__(self, 
                             outcomes=['normal'],
                             input_keys=['playing_counter_in'],
                             output_keys=['playing_counter_out'])
       

    def execute(self, userdata):
        """! Playing state execution 
        @section Description
        In this state is called the class image_feature in order to make the robot tracking the robot
        @return the normal state in case of absence of obstacle
        """
        global detected_object
        
        ## position messages for the rotation of the head of the robot
        msg_right = 0.7
        msg_left = -0.7
        msg_center = 0.0
       
        
        im = image_feature()

        print('start timer')
        ## variable used to define a timer to stop the execution of image_feature
        t_end = time.time() + 60 

        while time.time() < t_end :

                        ## subscriber to the topic object used to detect wheter there's an object or not
                        rospy.Subscriber('object', String, call_play)
                        ## publisher that publishes the messages position to rotate the head
                        pub_head_rot = rospy.Publisher("robot/joint1_position_controller/command", Float64, queue_size=1)
                        
                        #if the object is detected the robot keeps moving the head and initializes the timer
                        if detected_object == 1 :
                                pub_head_rot.publish(msg_right)
                                time.sleep(2)
                                pub_head_rot.publish(msg_left)
                                time.sleep(2)
                                pub_head_rot.publish(msg_center)
                                time.sleep(2)
                                print('start timer')
                                t_end = time.time() +60 
                                rospy.Subscriber('object', String, call_play)
                            
                         
                           
        cv2.destroyAllWindows()               
        print('no object found')
        time.sleep(2)
        return ('normal')    

        
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
        smach.StateMachine.add('RANDOMLYGOING', RandomlyGoing(), 
                               transitions={'normal':'RANDOMLYGOING', 
                                            'sleep':'SLEEPING',
					    'play':'PLAYING'},
                               remapping={'randomlygoing_counter_in':'sm_counter', 
                                          'randomlygoing_counter_out':'sm_counter'})

        smach.StateMachine.add('PLAYING', Playing(), 
                               transitions={'normal':'RANDOMLYGOING'
					    },
                                            
							
                               remapping={'playing_counter_in':'sm_counter',
                                          'plying_counter_out':'sm_counter'})

        
       
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

