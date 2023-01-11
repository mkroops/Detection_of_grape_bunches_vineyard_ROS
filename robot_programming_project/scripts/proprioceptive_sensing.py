#!/usr/bin/env python

#----------------------------------
#Author:Manigandan Sivalingam
#description: To tell about internal state of the robot, publish available memory to main node to 
#keep track of available memeory
#----------------------------------

#python lib
import psutil

#ros libraries
import rospy
from std_msgs.msg import String

class proprioceptive_sensing:
    
    '''
    inspired from workshop material
    Function name:__init__
    Description: initiate grape_bunch_counter/memory_info topic'''

    def __init__(self):
        
        self.mem_info = rospy.Publisher('grape_bunch_counter/memory_info', String, queue_size=10)
        self.rate = rospy.Rate(10) # 10hz
    
    '''
    inspired from workshop material
    Function name: memory_info
    Description: publish available memory and sensing about istself and publish it'''

    def memory_info(self):
        
        while not rospy.is_shutdown():
            info = psutil.virtual_memory()
            available_mem = info.available
            #publish available memory
            self.mem_info.publish(str(available_mem))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        #initialize grape_bunch_counter_proprioceptive_sensing node
        rospy.init_node('grape_bunch_counter_proprioceptive_sensing', anonymous=True)
        mem = proprioceptive_sensing()
        mem.memory_info()

    except rospy.ROSInterruptException:
        pass
