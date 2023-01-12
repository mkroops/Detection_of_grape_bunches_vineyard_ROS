#!/usr/bin/env python

#----------------------------------
#Author: Manigandan Sivalingam, workshop material, opensource
#description: To detect grape bunches by using open cv library and count the nymber of grape bunches in vineyard
#and update to main to main node, similary it captures images and send grape bunch coordinates to image saver node
#and main node respectively.
#----------------------------------

# Python libs
import sys, time
import numpy

# OpenCV
import cv2
from std_msgs.msg import Bool

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

class image_projection:
    

    camera_model = None
    image_depth_ros = None

    #set initially capture image as false
    capture_image = False

    #count variable and save grape bunch coordinates
    grape_bunch_count = 0
    grape_bunch_coordinates = []
    missing_coordinates_count = 0
    #set visualisation
    visualisation = True
    #count the no of images
    image_count = 1
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    # (84.1/1920) / (70.0/512)
    color2depth_aspect = (84.1/1920) / (70.0/512)

    def __init__(self):    
        
        #creating bridge between ros and cv
        self.bridge = CvBridge()

        #getting camera info #from workshop material
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        #publishing grape bunch location through X,Y,Z cordinates #from workshop material
        self.object_location_pub = rospy.Publisher('/grape_bunch_counter/object_location', PoseStamped, queue_size=10)

        #publish the captured image to save the image #author: Manigandan Sivalingam
        self.image_saver = rospy.Publisher('/grape_bunch_counter/image_saver', Image, queue_size=10)
        
        #receive input to capture image #author: Manigandan Sivalingam
        rospy.Subscriber('/grape_bunch_counter/capture_image', Bool, self.capture_image_status)

        #right color camera is used to stream the video #from workshop material
        rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
            Image, self.image_color_callback)

        #right depth camera is used to stream the video #from workshop material
        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect",
            Image, self.image_depth_callback)

        self.tf_listener = tf.TransformListener()

    '''
    Function name:capture_image_status
    Description: change capture status to true when callback is received to capture image'''

    def capture_image_status(self, data):
        self.capture_image = data.data

    #from workshop material
    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    #from workshop material
    def image_depth_callback(self, data):
        self.image_depth_ros = data

    #from workshop material
    def image_color_callback(self, data):
        
        #update initially grape bunch count as 0 when call back is recieved
        self.grape_bunch_count = 0
        centres = []
        
        #from workshop material
        map_coordinate = {}

        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # convert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)


        ''' this solution learned from stack over flow to solve my problem 
        convert bgr image to hsv image
        https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
        '''
        hsv_image = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)

        '''mask only grape by giving upper and lower threshold for grape bunches color
        trail and error attempts made to detect grape bunches'''
        image_mask = cv2.inRange(hsv_image,
                                 numpy.array((85, 10, 50)),
                                 numpy.array((145, 255, 255)))


        grape_bunch = cv2.bitwise_and(hsv_image, hsv_image, mask=image_mask)

        '''this solution has been solved by getting inspiration 
        from https://pub.towardsai.net/drawing-bounding-box-method-in-image-processing-ec7487393cfa'''
        
        h, s, v = cv2.split(grape_bunch)
        #do binary threshold
        ret, th1 = cv2.threshold(h, 180, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)

        kernel = numpy.ones((1,1), dtype = "uint8")/9
        bilateral = cv2.bilateralFilter(th1, 9 , 75, 75)
        erosion = cv2.erode(bilateral, kernel, iterations = 1)

        #find noise and remove it from edge
        '''the connected component method in the image it separates the white and black objects in the image. 
        We just want the white pixel object to be removed and neglect the black pixels.'''
        pixel_components, output, stats, centroids = cv2.connectedComponentsWithStats(erosion, connectivity=8)
        area = stats[1:, -1]; pixel_components = pixel_components - 1
        
        #trail and attempts made in min size value to get desired output
        min_size = 50
        #Removing the small white pixel area below the minimum size
        image_eroded = numpy.zeros((output.shape))
        for i in range(0, pixel_components):
            if area[i] >= min_size:
                image_eroded[output == i + 1] = 255

        image_eroded = image_eroded.astype(numpy.uint8)
        
        #inspired from https://www.geeksforgeeks.org/erosion-dilation-images-using-opencv-python/
        #dilation is done because it is useful in joining broken parts of an object
        image_dilated = cv2.dilate(image_eroded, numpy.ones((15, 15)), iterations = 1)

        #inspired from workshop material and passed my dilated image to get contours
        contours, hierachy = cv2.findContours(
            image_dilated,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)
        
        #check each contours in dilated image
        for c in contours:
            a = cv2.contourArea(c)
            #inspired from https://stackoverflow.com/questions/44588279/find-and-draw-the-largest-contour-in-opencv-on-a-specific-color-python
            #trail and attempts made to play with contour area
            if a > 30.0:
                cv2.drawContours(image_color, c, -1, (255, 0, 0), 3)

                # calculate moments of the binary image
                moments = cv2.moments(c)
                
                #no bunches is detected if moments is 0
                if moments["m00"] == 0:
                    print('No grape bunches is detected.')
                    continue
                
                #take image coordinates of 2D image by using moments #inspired from workshop material
                image_coords = (moments["m01"] / moments["m00"], moments["m10"] / moments["m00"])
                #take depth coordinates of 2D image by using moments #inspired from workshop material
                depth_coords = [image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
                    image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect]
                
                #update the centres of grape bunches in list
                centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))
                
                #author: Manigandan Sivalingam
                #x pixel should not go beyond 424, just updating inbetween ranges
                if depth_coords[0] > 424:
                    continue
                #x pixel should not go beyond 512, just updating inbetween ranges
                if depth_coords[1] > 512:
                    continue

                #inspired from workshop material
                depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

                #author: Manigandan Sivalingam
                #nan values are spotted just trying fo next pixel to avoid nan values
                next_pixel = [0,-1, -2, 1, 2] #add or subract by these values
                if (numpy.isnan(depth_value)):
                    for x1 in next_pixel:
                        for y1 in next_pixel: 
                            #change untill nan values are removed
                            new_depth_coords_x1 = int(depth_coords[0]) + x1
                            new_depth_coords_y1 = int(depth_coords[1]) + y1
                            depth_value = image_depth[int(new_depth_coords_x1), int(new_depth_coords_y1)]
                            if numpy.isnan(depth_value):
                                continue
                            else:
                                #update new depth coordinates
                                depth_coords[0] = new_depth_coords_x1
                                depth_coords[1] = new_depth_coords_y1
                                break 
                
                #convert 2d coordinates to 3d coordinates, inspired from workshop material
                camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) 
                camera_coords = [x/camera_coords[2] for x in camera_coords] 
                camera_coords = [x*depth_value for x in camera_coords] 

                #update object location, inspired from workshop material
                object_location = PoseStamped()
                object_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"
                object_location.pose.orientation.w = 1.0
                object_location.pose.position.x = camera_coords[0]
                object_location.pose.position.y = camera_coords[1]
                object_location.pose.position.z = camera_coords[2]

                #publish grape bunch location, inspired from workshop material
                self.object_location_pub.publish(object_location)

                #transform x,y,z coordinates in map to spot where is grape bunch, inspired from workshop material
                p_camera = self.tf_listener.transformPose('map', object_location)

                xyz = []
                xyz.append(p_camera.pose.position.x)
                xyz.append(p_camera.pose.position.y)
                xyz.append(p_camera.pose.position.z)
                #update grape bunches coordinates in map and append in list, inspired from workshop material
                map_coordinate[self.grape_bunch_count] = xyz
                eval = numpy.isnan(p_camera.pose.position.x)
                #count the grape bunches  #author: Manigandan Sivalingam
                if eval:
                    self.missing_coordinates_count = self.missing_coordinates_count + 1
                else: 
                    self.grape_bunch_count = self.grape_bunch_count + 1
                    #for visulaisation purpose
                    if self.visualisation:
                        cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
                        cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)

        cv2.imshow("image depth", image_depth)
        cv2.imshow("image color", image_color)
        cv2.waitKey(1)

        #capture image and publish to image saver node where all images are saved.  #author: Manigandan Sivalingam
        if self.capture_image == True:
            #differnt types of images are going to publish to image saver node #author: Manigandan Sivalingam
            images = [image_color, image_depth, image_dilated, grape_bunch]
            for image in images:
                #convert cv2 format to ros message in order to publish.  #author: Manigandan Sivalingam
                self.image_saver.publish(self.bridge.cv2_to_imgmsg(image))
            
            #update grape bunch coordinate for every image,  #author: Manigandan Sivalingam
            self.grape_bunch_coordinates.append(map_coordinate)
            #now capture image is set to false
            self.capture_image = False
            self.image_count = self.image_count + 1

            #debugging purpose log info of grape bunch coordinates,  #author: Manigandan Sivalingam
            for key, value in map_coordinate.items():
                print(key, value)
            print("Count", self.grape_bunch_count)

    #send grape bunch cordinates to main node  #author: Manigandan Sivalingam    
    def get_grapes_coordinates(self):
        return self.grape_bunch_coordinates

def main(args):
    #Initializes grape_bunch_counter_image_projection and cleanup ros node
    rospy.init_node('grape_bunch_counter_image_projection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
