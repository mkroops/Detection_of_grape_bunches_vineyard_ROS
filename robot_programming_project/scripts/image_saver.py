#!/usr/bin/env python

#----------------------------------
#Author: Manigandan Sivalingam
#description: To save the taken images to the image folder by subscibing to topicL /grape_bunch_counter/image_saver
#----------------------------------

#python lib
import os

# OpenCV
import cv2

#rospy lib and messages
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class image_saver:

    #initially set image as none and image count 1
    image = None
    image_count = 1
    #count no of files
    file_count = 0
    max_file_count = 4
    #name of image files
    file_names = ["Contours", "Depth_image", "Dilated_image", "Masking"]
    
    #author: Manigandan Sivalingam
    #initiate CV bridge and subscribe to topic:/grape_bunch_counter/image_saver
    def __init__(self):

        #initiate bridge for ROS and cv2
        self.bridge = CvBridge()
        rospy.Subscriber("/grape_bunch_counter/image_saver", Image, self.save_image)
        #get directory path which the file is present
        dirname, filename = os.path.split(os.path.abspath(__file__))
        #go back to current file directory and set the path to images folder
        self.file_path = os.path.normpath(dirname + os.sep + os.pardir) + "/images/"
    
    '''
    author: Manigandan Sivalingam
    Function name:save_image
    Description: save the images which is published by image_projection when it is reached to the desired waypoint'''

    def save_image(self, image):
        #format for file name
        file_name = self.file_path + self.file_names[self.file_count] + "_" + str(self.image_count)+".jpeg"
        self.file_count = self.file_count + 1
        #convert ros message image format to cv2 to write the image
        self.image = self.bridge.imgmsg_to_cv2(image)
        cv2.imwrite(file_name, self.image)
        print("Image Saved")

        #Four types of images will be stored for each capture
        if self.file_count == self.max_file_count:
            self.image_count = self.image_count + 1   
            self.file_count = 0         

if __name__ == '__main__':
    try:
        #intiate node called grape_bunch_counter_image_saver and same name node cannot be used
        rospy.init_node('grape_bunch_counter_image_saver', anonymous=True)
        image_saver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
