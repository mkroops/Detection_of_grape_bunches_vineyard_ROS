#! /usr/bin/env python
# ----------------------------------
# @author: gpdas
# @email: pdasgautham@gmail.com
# @date:
# ----------------------------------

import rospy
import actionlib
from time import sleep

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

rospy.init_node('topological_navigation_client', anonymous=True)
client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
client.wait_for_server()

class set_topo_nav_goal:
    
    def move_to_topological_node(self, waypoint):
        
        goal = GotoNodeGoal()
        goal.target = waypoint
        rospy.loginfo("going to %s", goal.target)
        client.send_goal(goal)
        status = client.wait_for_result() # wait until the action is complete
        result = client.get_result()
        rospy.loginfo("status is %s", status)
        rospy.loginfo("result is %s", result)

