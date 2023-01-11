#!/usr/bin/env python

#----------------------------------
#Author: https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
#Description: get odometry data and it helps robot to rotate to desired angle.
#----------------------------------

# Python libs
import math
import sys, rospy
from time import sleep

# Ros libraries and messages
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion 


class move_robot():
    
    #inially set controller to 0.5
    kP = 0.5
    #inially set roll, pitch, yaw to 0.0
    roll = pitch = yaw = 0.0
    
    '''
    inspired from https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
    Function name:__init__
    Description: initiate cmd velocity and odometry topics'''
    def __init__(self):
        
        self.publisher = rospy.Publisher('/thorvald_001/teleop_joy/cmd_vel',Twist, queue_size=1)
        rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, self.odom)
        self.rate = rospy.Rate(10)

    '''
    inspired from https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
    Function name: odom
    Description: convert quaternion to euler'''

    def odom(self, odom_data):

        robot_position = odom_data.pose.pose
        quaternion = (robot_position.orientation.x, robot_position.orientation.y, 
                      robot_position.orientation.z, robot_position.orientation.w)
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(quaternion)

    '''
    inspired from https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/
    Function name: rotate
    Description: robot rotate to desired angle'''

    def rotate(self, target_angle):
        
        #set angle
        max_angle = math.pi / target_angle
        t = Twist()
        #get radian
        target_rad = target_angle * math.pi/target_angle
        
        while True:
            if math.fabs(target_rad - self.yaw) > max_angle:
                t.angular.z = self.kP  * (target_rad - self.yaw)
                #publish cmd velocity to move robot
                self.publisher.publish(t)
            else:
                return

if __name__ == '__main__':
    try:
        #initiate grape_bunch_counter_move_robot node
        rospy.init_node('grape_bunch_counter_move_robot', anonymous=True)
        move_robot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass