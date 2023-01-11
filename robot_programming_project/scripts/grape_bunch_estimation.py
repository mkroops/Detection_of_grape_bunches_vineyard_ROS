#!/usr/bin/env python

#----------------------------------
#Author:Manigandan Sivalingam
#description: To estimate the grape bunches in vineyard and remove dupicate count from same and different images.
#----------------------------------

#python lib
from time import sleep
from enum import Enum

#open cv
import sys, rospy
from std_msgs.msg import Int64

class grape_bunch_estimation:
    # X, Y, Z coordinates
    axis_xyz = ['x', 'y', 'z']
    #set flag for same grape bunches
    same_grape_bunches = False
    #variables for duplicate count
    duplicate_bunches_count = 0
    image_grape_bunch_count = []
    duplicate_count_same_image = [] 
    duplicate_count_diff_image = []
    image_1_4_grape_bunches =  image_2_3_grape_bunches = 0
    overall_grape_bunches = 0
    #fixed 4 images
    images = [0, 1, 2, 3]
    #tolerence level for x,y,z coordinates 0.1 for same image and 0.5 for different images
    tolerence_level = [0.1, 0.5]

    #initialize topics in init function
    def __init__(self):
        #publish grape count after estimating the grape bunches
        self.grape_bunches_count = rospy.Publisher('grape_bunch_counter/get_grape_bunches_count', Int64, queue_size=10)

    '''
    author: Manigandan Sivalingam
    Function name:remove_duplicates
    Description: remove double count for same image and for different image from grape bunches coordinates
    same image tolerence level set to 0.1 this include for x,y,z values
    different image tolerence level set to 0.5 this include for x,y,z values'''
    
    def remove_duplicates(self, grapes_coordinates, image_1, image_2, tolerence_level):
        
        #initially duplicate count set to 0.
        self.duplicate_bunches_count = 0

        #check each grape bunch coordinates with other grape bunch coordinates
        for grape_bunch_1 in range(len(grapes_coordinates[image_1])):
            
            #add and subract tolerance level to set max and min boundry
            self.img_1_max_boundry = {}
            self.img_1_min_boundry = {}
            image_1_grapes_coords = grapes_coordinates[image_1]
            self.same_grape_bunches = False
            
            #update max and minimum boundry for every axis
            for axis in range(len(self.axis_xyz)):
                self.img_1_max_boundry[self.axis_xyz[axis]] =  image_1_grapes_coords[grape_bunch_1][axis] + tolerence_level
                self.img_1_min_boundry[self.axis_xyz[axis]] =  image_1_grapes_coords[grape_bunch_1][axis] - tolerence_level
            
            #check other grape bunch falls in between boundry level
            for grape_bunch_2 in range(len(grapes_coordinates[image_2])):
                image_2_grapes_coords = grapes_coordinates[image_2]
                
                #same grape bunch ignore
                if (grape_bunch_1 == grape_bunch_2) and (image_1 == image_2):
                    continue

                #if other grape bunch falls inbetween boundry level then its same grape bunch or duplicate count
                if ((self.img_1_min_boundry[self.axis_xyz[2]] <= image_2_grapes_coords[grape_bunch_2][2] 
                                  <= self.img_1_max_boundry[self.axis_xyz[2]]) and 
                    (self.img_1_min_boundry[self.axis_xyz[1]] <= image_2_grapes_coords[grape_bunch_2][1] 
                                  <= self.img_1_max_boundry[self.axis_xyz[1]]) and
                    (self.img_1_min_boundry[self.axis_xyz[0]] <= image_2_grapes_coords[grape_bunch_2][0] 
                                  <= self.img_1_max_boundry[self.axis_xyz[0]])):
                        print("Images", image_1, grape_bunch_1, grape_bunch_2)
                        print(image_1_grapes_coords[grape_bunch_1], image_2_grapes_coords[grape_bunch_2])
                        #set flag duplicate count as true
                        self.same_grape_bunches = True
            
            #count the number of duplicate grape bunches
            if self.same_grape_bunches == True:
                self.duplicate_bunches_count =  self.duplicate_bunches_count + 1
        print("duplicate", self.duplicate_bunches_count)
        return self.duplicate_bunches_count

    '''
    author: Manigandan Sivalingam
    Function name:estimate_grape_bunch_count
    Description: estimate grape bunch by adding all images count and subracting duplicate count from all images'''
    
    def estimate_grape_bunch_count(self, grapes_coordinates):

        #check duplicate count for same images    
        for image in self.images:
            self.duplicate_count_same_image.append(int((self.remove_duplicates
                (grapes_coordinates, image, image, self.tolerence_level[0])) / 2))
            self.image_grape_bunch_count.append(len(grapes_coordinates[image]))

        #check duplicate count for different images
        #compare image 1 and image 4
        self.duplicate_count_diff_image.append(int((self.remove_duplicates
            (grapes_coordinates, self.images[0], self.images[3], self.tolerence_level[1]))))
        #compare image 2 and image 3
        self.duplicate_count_diff_image.append(int((self.remove_duplicates
            (grapes_coordinates, self.images[1], self.images[2], self.tolerence_level[1]))))
        #compare image 1 and image 2
        self.duplicate_count_diff_image.append(int((self.remove_duplicates
            (grapes_coordinates, self.images[0], self.images[1], self.tolerence_level[1]))))
        #compare image 3 and image 4
        self.duplicate_count_diff_image.append(int((self.remove_duplicates
            (grapes_coordinates, self.images[2], self.images[3], self.tolerence_level[1]))))
        

        self.image_1_4_grape_bunches = ((self.image_grape_bunch_count[0] + self.image_grape_bunch_count[3])
                                         - self.duplicate_count_diff_image[0])
        self.image_2_3_grape_bunches = ((self.image_grape_bunch_count[1] + self.image_grape_bunch_count[2]) 
                                         - self.duplicate_count_diff_image[1])
        
        #estimate grape bunch by adding all images count and subracting duplicate count from all images
        self.overall_grape_bunches = ((self.image_1_4_grape_bunches + self.image_2_3_grape_bunches) - 
                                ( self.duplicate_count_same_image[0] + self.duplicate_count_same_image[1] 
                                + self.duplicate_count_same_image[2] + self.duplicate_count_same_image[3])-
                                (self.duplicate_count_diff_image[2] + self.duplicate_count_diff_image[3]))

        #publish over all grape count to topic: grape_bunch_counter/get_grape_bunches_count
        self.grape_bunches_count.publish(self.overall_grape_bunches)

    '''
    author: Manigandan Sivalingam
    Function name:grape_bunch_log_info
    Description: display the log info for grape bunches count'''
    
    def grape_bunch_log_info(self):

        for i in range(len(self.images)):
            print("Image {} Count = {}".format(i+1 , self.image_grape_bunch_count[i]))
            print("Duplicate Count Same image {} = {}".format(i+1, self.duplicate_count_same_image[i]))
        
        print("Duplicate count Image 1 and 4 =", self.duplicate_count_diff_image[0])
        print("Duplicate count Image 2 and 3 =", self.duplicate_count_diff_image[1])
        print("Duplicate count Image 1 and 2 =", self.duplicate_count_diff_image[2])
        print("Duplicate count Image 3 and 4 =", self.duplicate_count_diff_image[3])
        print("Total Count Image 1 and 4 =", self.image_1_4_grape_bunches)
        print("Total Count Image 2 and 3 =", self.image_2_3_grape_bunches)
        print("Overall Grape Count =", self.overall_grape_bunches)

if __name__ == '__main__':
    try:
        #initiate node called grape_bunch_counter_estimation and same name node cannot be used
        rospy.init_node('grape_bunch_counter_estimation', anonymous=True)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass