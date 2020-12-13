#!/usr/bin/env python

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



####################################### for the navigation #######################################

message = " "
flag_play = 0
stop = 0


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



####################################### for tracking the ball #######################################
VERBOSE = False
detected_object = 0

def call_play(data):
   	 	
            global detected_object
            
            message = data.data
            #checking that there's actually a message from the publisher
            if message == "1" :
                detected_object = 1
                
            else :
                detected_object = 0
            #print('end_proc = ' ,detected_object)
def call_stop(data):
   	 	
            global stop
            
            msg = data.data
            #checking that there's actually a message from the publisher
            if msg == "1" :
                stop = 1
                
            else :
                stop = 0

class image_feature:
    global detected_object,detected_object #counter_time
    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
       
     # topic where we publish
        self.image_pub = rospy.Publisher("robot/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("robot/cmd_vel",
                                       Twist, queue_size=1)
        
        self.subscriber = rospy.Subscriber("robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)
        
        #self.pub_play = rospy.Publisher('/robot/object', String, queue_size=10) 
        #self.pub_stop = rospy.Publisher('play_stop', String, queue_size=10) 
        
    def callback(self, ros_data):
        
        detected_object = 0
        pub_play = rospy.Publisher('/robot/object', String, queue_size=10)
        pub_stop = rospy.Publisher('play_stop', String, queue_size=10) 
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
                pub_play.publish(msg)
                
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
                if abs(vel.linear.x) < 0.01 :
                    stop_msg = "1"
                    pub_stop.publish(stop_msg)
                    vel.angular.z = 0
                    vel.linear.x = 0
                    self.vel_pub.publish(vel)

        else: # if I'm not detecting obj, I rotate in order to find it
                msg = "0"
                pub_play.publish(msg)
                vel = Twist()
                #rate = rospy.Rate(20)
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
                
        cv2.imshow('window', image_np)
            
        cv2.waitKey(2)
        
            
       
        
####################################### state machine functions ##########################

def callback_play(data):
   	 	
		global flag_play
		global message
		message = data.data
		#checking that there's actually a message from the publisher
		if message == "start play" :
			flag_play = 1
		else :
			flag_play = 0
		

def user_action():
			
    global flag_play 
    global message

    rospy.Subscriber("/ball/chatter", String, callback_play)
    #print(flag_play)
    if flag_play == 1 :
        #flag_play = 0
        return('play')
    else : 
        return random.choice(['normal','sleep'])
				

# define state RandomlyGoing
class RandomlyGoing(smach.State):
    def __init__(self):
	
	
        smach.State.__init__(self, 
                             outcomes=['sleep','normal','play'],
                             input_keys=['randomlygoing_counter_in'],
                             output_keys=['randomlygoing_counter_out'])
        
    def execute(self, userdata):
    
        
        rospy.Subscriber("/ball/chatter", String, callback_play)

        if flag_play != 1 :
                
                #print('flag = ' ,flag_play)
                #cv2.destroyAllWindows()  
                 #where the robot fix its position in order to reach the target
                client = actionlib.SimpleActionClient('/robot/reaching_goal', exp_assignment2.msg.PlanningAction)

    
                client.wait_for_server()

                
                goal = exp_assignment2.msg.PlanningGoal()
                goal.target_pose.pose.position.x = -5
                goal.target_pose.pose.position.y = 5
            
                client.send_goal(goal)

                
                client.wait_for_result()

                #return client.get_result()  # A FibonacciResult
                return user_action()
                
        return ('play')
        

        rospy.loginfo('Executing state RANDOMLYGOING (users = %f)'%userdata.randomlygoing_counter_in)
        userdata.randomlygoing_counter_out = userdata.randomlygoing_counter_in + 1
	
        
    

# define state Sleeping
class Sleeping(smach.State):
    def __init__(self):
	

        smach.State.__init__(self, 
			     outcomes=['play','normal','sleep'],
                             input_keys=['sleeping_counter_in'],
                             output_keys=['sleeping_counter_out'])
        
    def execute(self, userdata):

        rospy.Subscriber("/ball/chatter", String, callback_play)
        if flag_play != 1 :
            
            
            client = actionlib.SimpleActionClient('/robot/reaching_goal', exp_assignment2.msg.PlanningAction)

    
            client.wait_for_server()

                
            goal = exp_assignment2.msg.PlanningGoal()
            goal.target_pose.pose.position.x = -6
            goal.target_pose.pose.position.y = 8
            
            client.send_goal(goal)

                
            client.wait_for_result()
            time.sleep(10)
            #return client.get_result()  # A FibonacciResult
            return user_action()
                
        return ('play')
   	
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
        global detected_object,stop
        #cv2.destroyAllWindows() 
        msg_right = 0.7
        msg_left = -0.7
        msg_center = 0.0
        print(detected_object)
        rate = rospy.Rate(20)
        #while not rospy.is_shutdown():
        im = image_feature()
        print('inizio tempo')
        t_end = time.time() + 10 # 60
        while time.time() < t_end :

                        #im = image_feature()
                        rospy.Subscriber('object', String, call_play)
                        pub_head_rot = rospy.Publisher("robot/joint1_position_controller/command", Float64, queue_size=1)
                        
                            
                        if detected_object == 1 :
                                
                                print('inizio tempo')
                                t_end = time.time() +10 # 60
                                rospy.Subscriber('object', String, call_play)
                                rospy.Subscriber('play_stop', String, call_stop)
                                if stop == 1 :
                                        pub_head_rot.publish(msg_right)
                                        time.sleep(5)
                                        pub_head_rot.publish(msg_left)
                                        time.sleep(5)
                                        pub_head_rot.publish(msg_center)
                                        #time.sleep(5)
                        
                        #rate.sleep() 
                           
        cv2.destroyAllWindows()               
        print('no object found')
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
                               transitions={'normal':'RANDOMLYGOING', 
                                            'sleep':'SLEEPING',
					    'play':'PLAYING'},
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

