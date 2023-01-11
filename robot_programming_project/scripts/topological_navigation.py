#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @description: To move to desired waypoint and navigate autonomously by topological navigation
# ----------------------------------

#python libs
from time import sleep

#rospy libs
import rospy
import actionlib
from std_msgs.msg import String
from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

class topological_navigation:
    
    '''
    @author: gpdas
    Function name:  __init__
    Description: initializes simple action /thorvald_001/topological_navigation'''
    
    def __init__(self):

        self.client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        self.client.wait_for_server()
    
    '''
    @author: gpdas
    Function name:  move_to_topological_node
    Description: move to desired waypoint in the map'''

    def move_to_topological_node(self, waypoint):
    
        goal = GotoNodeGoal()
        goal.target = waypoint
        rospy.loginfo("going to %s", goal.target)
        #send goal
        self.client.send_goal(goal)
        status = self.client.wait_for_result() # wait until the action is complete
        result = self.client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)


if __name__ == '__main__':
    try:
        #intializes grape_bunch_counter_topological_navigation_client node
        rospy.init_node('grape_bunch_counter_topological_navigation_client', anonymous=True)
        topological_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

