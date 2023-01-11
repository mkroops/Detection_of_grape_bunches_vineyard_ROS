#!/usr/bin/env python

#----------------------------------
#Author: Manigandan Sivalingam
#description: act as a main node / master where collects all information from other nodes and run the state
#machine accordingly 

# OpenCV
import cv2

# Python libs
from time import sleep
from enum import Enum

# Ros libraries and messages
import sys, rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from image_projection import image_projection
from move_robot import move_robot
from grape_bunch_estimation import grape_bunch_estimation
from topological_navigation import topological_navigation

#tells the state of the robot, state machine concept is inspired #author: Manigandan Sivalingam
class robot_state(Enum):
    START = 0 #start 
    MOVE_TO_WAY_POINT = 1 #move to all waypoints which is mentioned below
    TAKE_IMAGE = 2 #take image
    IMAGE_TAKEN = 3 #update imgae has taken
    ROTATE_TO_VINEYARD = 4 #rotate to vineyard
    ESTIMATE_GRAPE_BUNCHES = 5 #estimate total grape bunches
    STOP = 6 #stop and exit

#7 waypoints has pointed in map to move autonomously
#topological navigation has been used
class way_point(Enum):
    WAY_POINT_0 = 0
    WAY_POINT_1 = 1
    WAY_POINT_2 = 2
    WAY_POINT_3 = 3
    WAY_POINT_4 = 4
    WAY_POINT_5 = 5
    WAY_POINT_6 = 6

class run:
    
    #update internal state of robot that is available memory
    update_mem_info = ''
    #update grape bunches count to main node
    grape_bunches_count = 0

    '''
    author: Manigandan Sivalingam
    Function name:__init__
    Description: initiate all objects and set robot initial state and initial waypoint'''
    def __init__(self):
        
        self.image_projection = image_projection()
        self.move_robot = move_robot()
        self.topo_navigation = topological_navigation()
        self.estimate_grape_bunch = grape_bunch_estimation()
        self.current_state = robot_state.START
        self.current_way_point = way_point.WAY_POINT_0

        #publish capture image to image projection node
        self.capture_image = rospy.Publisher('/grape_bunch_counter/capture_image', Bool, queue_size=1)
        #get memory info from proprioceptive sensing node
        rospy.Subscriber('grape_bunch_counter/memory_info', String, self.update_memory_info)
        #get overall grape bunches from grape bunch estimation node
        rospy.Subscriber('grape_bunch_counter/get_grape_bunches_count', Int64 , self.get_grape_count)
    
    '''
    author: Manigandan Sivalingam
    Function name:update_memory_info
    Description: update memory info to main node'''

    def update_memory_info(self, data):
        self.update_mem_info = data

    '''
    author: Manigandan Sivalingam
    Function name:get_grape_count
    Description: update grape bunches count to main node'''

    def get_grape_count(self, data):
        self.grape_bunches_count = data.data

    '''
    author: Manigandan Sivalingam
    Function name:run
    Description: run all the states'''

    def run(self):

        while True:

            #initiate and update memoru info    
            if self.current_state is robot_state.START:
                print("START")
                sleep(10)
                print("Available Memory is:", self.update_mem_info)
                self.current_state = robot_state.MOVE_TO_WAY_POINT

            #move to desired waypoint
            elif self.current_state is robot_state.MOVE_TO_WAY_POINT:
                
                #in waypoint, move to way point 0
                if self.current_way_point is way_point.WAY_POINT_0:
                    self.topo_navigation.move_to_topological_node("WayPoint0")
                    #wait for 10s to get proper angle to get images
                    sleep(10)
                    self.current_state = robot_state.TAKE_IMAGE
                    self.current_way_point = way_point.WAY_POINT_1

                #in waypoint, move to way point 1
                elif self.current_way_point is way_point.WAY_POINT_1:
                    self.topo_navigation.move_to_topological_node("WayPoint1")
                    #wait for 10s to get proper angle to get images
                    sleep(10)
                    self.current_state = robot_state.TAKE_IMAGE
                    self.current_way_point = way_point.WAY_POINT_2

                #in waypoint, move to way point 2
                elif self.current_way_point is way_point.WAY_POINT_2:
                    self.topo_navigation.move_to_topological_node("WayPoint2")
                    self.current_way_point = way_point.WAY_POINT_3
                
                #in waypoint, move to way point 3
                elif self.current_way_point is way_point.WAY_POINT_3:
                    self.topo_navigation.move_to_topological_node("WayPoint3")
                    self.current_way_point = way_point.WAY_POINT_4

                #in waypoint, move to way point 4
                elif self.current_way_point is way_point.WAY_POINT_4:
                    self.topo_navigation.move_to_topological_node("WayPoint4")
                    #rotate 180, since robot is using right camera
                    self.move_robot.rotate(180)
                    #wait for 10s to get proper angle to get images
                    sleep(10)
                    self.current_state = robot_state.TAKE_IMAGE
                    self.current_way_point = way_point.WAY_POINT_5
                
                #in waypoint, move to way point 5
                elif self.current_way_point is way_point.WAY_POINT_5:
                    self.topo_navigation.move_to_topological_node("WayPoint5")
                    #rotate 180, since robot is using right camera
                    self.move_robot.rotate(180)
                    #wait for 10s to get proper angle to get images
                    sleep(10)
                    self.current_state = robot_state.TAKE_IMAGE
                    self.current_way_point = way_point.WAY_POINT_6
                
                #in waypoint, move to way point 6
                elif self.current_way_point is way_point.WAY_POINT_6:
                    self.topo_navigation.move_to_topological_node("WayPoint6")
                    #after completing all waypoints, move to state estimate grape bunches
                    self.current_state = robot_state.ESTIMATE_GRAPE_BUNCHES
            
            elif self.current_state is robot_state.TAKE_IMAGE:
                #send command to image projection node to take image
                self.capture_image.publish(True)
                sleep(5)
                #after takin image, change state to image taken
                self.current_state = robot_state.IMAGE_TAKEN

            elif self.current_state is robot_state.IMAGE_TAKEN:
                #if image taken move to waypoint
                self.current_state = robot_state.MOVE_TO_WAY_POINT

            #after completing calculate and remove all duplicate grape bunches and gives final grape bunch count
            elif self.current_state is robot_state.ESTIMATE_GRAPE_BUNCHES:
                #get grape bunch coordinates from image projection node
                grapes_coordinates = self.image_projection.get_grapes_coordinates()
                for map_coordinate in grapes_coordinates:
                    for key, value in map_coordinate.items():
                        print(key, value)

                self.estimate_grape_bunch.estimate_grape_bunch_count(grapes_coordinates)
                #display log info of grape bunch estimation
                self.estimate_grape_bunch.grape_bunch_log_info()
                sleep(1)
                print("Total Grape Bunches:", self.grape_bunches_count)
                #move to stop state
                self.current_state = robot_state.STOP      

            #destroy all windows and exit
            elif self.current_state is robot_state.STOP:
                cv2.destroyAllWindows()
                exit()
    
if __name__ == '__main__':
    try:
        rospy.init_node('main', anonymous=True)
        Run = run()
        #run the function
        Run.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
