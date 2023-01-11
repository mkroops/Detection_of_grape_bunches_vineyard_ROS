#!/usr/bin/env python

# Python libs
import sys, time
import numpy
# OpenCV
import cv2

# Ros libraries
import roslib, rospy, image_geometry, tf

# Ros Messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError


way_point = "stop"
grapes_coordinates = []
image_count = 1
g_nan_count = 0

class image_projection:
    camera_model = None
    image_depth_ros = None

    visualisation = True
    # aspect ration between color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width) from the kinectv2 urdf file
    # (84.1/1920) / (70.0/512)
    color2depth_aspect = (84.1/1920) / (70.0/512)

    def __init__(self):    
        #rospy.init_node('image_projection', anonymous=True)
        self.bridge = CvBridge()

        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', 
            CameraInfo, self.camera_info_callback)

        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)

        rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect",
            Image, self.image_color_callback)

        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect",
            Image, self.image_depth_callback)

        self.tf_listener = tf.TransformListener()

    def camera_info_callback(self, data):
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.camera_info_sub.unregister() #Only subscribe once

    def image_depth_callback(self, data):
        self.image_depth_ros = data

    def image_color_callback(self, data):
        # wait for camera_model and depth image to arrive
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        # covert images to open_cv
        try:
            image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        except CvBridgeError as e:
            print(e)

        hsv_img = cv2.cvtColor(image_color, cv2.COLOR_BGR2HSV)
        image_mask = cv2.inRange(hsv_img,
                                 numpy.array((85, 10, 50)),
                                 numpy.array((145, 255, 255)))


        #cv2.imshow("Mask", image_mask)
        #cv2.imshow("Original Color", image_color)
        grape_bunch = cv2.bitwise_and(hsv_img, hsv_img, mask=image_mask)

        h, s, v = cv2.split(grape_bunch)
        #cv2.imshow("h", h)
        ret, th1 = cv2.threshold(h,180,255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        kernel = numpy.ones((1,1), dtype = "uint8")/9
        bilateral = cv2.bilateralFilter(th1, 9 , 75, 75)
        erosion = cv2.erode(bilateral, kernel, iterations = 1)
        
        pixel_components, output, stats, centroids = cv2.connectedComponentsWithStats(erosion, connectivity=8)
        area = stats[1:, -1]; pixel_components = pixel_components - 1
        min_size = 50
        img2 = numpy.zeros((output.shape))#Removing the small white pixel area below the minimum size
        for i in range(0, pixel_components):
            if area[i] >= min_size:
                img2[output == i + 1] = 255
        #cv2.imshow("shape", img2)

        img2_updated = img2.astype(numpy.uint8)
        img2_updated = cv2.dilate(img2_updated, numpy.ones((15, 15)), iterations = 1)

        hsv_contours, hierachy = cv2.findContours(
            img2_updated,
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        #img2_updated = cv2.resize(img2_updated, (0,0), fx=0.5, fy=0.5)
        #cv2.imshow("dilation", img2_updated)

        count = 0
        nan_count = 0
        centres = []
        image_coords = ()
        map_coordinate = {}
        image_color_vis = image_color
        image_depth_vis = image_depth
        
        for c in hsv_contours:
            #print("contours Length", len(hsv_contours))
            a = cv2.contourArea(c)
            if a > 20.0:
                cv2.drawContours(image_color_vis, c, -1, (255, 0, 0), 3)
                cv2.drawContours(image_depth_vis, c, -1, (255, 0, 0), 3)
            moments = cv2.moments(c)
            if moments["m00"] == 0:
                print('No object detected.')
                continue
            image_coords = (moments["m01"] / moments["m00"], moments["m10"] / moments["m00"])
            #print("no", count)
            image_coords_copy = (int(image_coords[0]), int(image_coords[1]))
            centres.append((int(moments['m10']/moments['m00']), int(moments['m01']/moments['m00'])))

            # calculate moments of the binary image
            M = cv2.moments(image_mask)

            if M["m00"] == 0:
                print('No object detected.')
                return
            #print("M00-", M["m00"])
            # calculate the y,x centroid
            #image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])
            # "map" from color to depth image
            depth_coords = [image_depth.shape[0]/2 + (image_coords[0] - image_color.shape[0]/2)*self.color2depth_aspect, 
                image_depth.shape[1]/2 + (image_coords[1] - image_color.shape[1]/2)*self.color2depth_aspect]
            # get the depth reading at the centroid location
            #print('image coords: ', image_coords)
            #print('depth coords: ', depth_coords)
            if depth_coords[0] > 424:
                depth_coords[0] = 423
                continue
            if depth_coords[1] > 512:
                depth_coords[1] = 511
                continue
            depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
            #print('depth value: ', depth_value)


            camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) 
            camera_coords = [x/camera_coords[2] for x in camera_coords] 
            camera_coords = [x*depth_value for x in camera_coords] 

            object_location = PoseStamped()
            object_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"
            object_location.pose.orientation.w = 1.0
            object_location.pose.position.x = camera_coords[0]
            object_location.pose.position.y = camera_coords[1]
            object_location.pose.position.z = camera_coords[2]

            self.object_location_pub.publish(object_location)
            p_camera = self.tf_listener.transformPose('map', object_location)

            xyz = []
            xyz.append(p_camera.pose.position.x)
            xyz.append(p_camera.pose.position.y)
            xyz.append(p_camera.pose.position.z)
            map_coordinate[count] = xyz
            eval = numpy.isnan(p_camera.pose.position.x)
            if eval:
                nan_count = nan_count + 1
                color = (0, 255, 0)
                image_color = cv2.putText(image_color, "nan", centres[-1], cv2.FONT_HERSHEY_SIMPLEX, 
                1, color , 2, cv2.LINE_AA)
            else: 
                color = (0, 0, 255)
                image_color = cv2.putText(image_color, str(count), centres[-1], cv2.FONT_HERSHEY_SIMPLEX, 
                    1, color , 2, cv2.LINE_AA)
                image_depth = cv2.putText(image_depth, str(count), centres[-1], cv2.FONT_HERSHEY_SIMPLEX, 
                    1, color , 2, cv2.LINE_AA)
                #print(centres)
                cv2.circle(image_color, centres[-1], 3, (0, 0, 0), -1)
                cv2.circle(image_depth, centres[-1], 3, (0, 0, 0), -1)
                #map_coordinate.append(xyz)
                count = count + 1
                
                #print(count, xyz)
                color = (0, 0, 255)
                if self.visualisation:
                    # draw circles
                    cv2.circle(image_color, (int(image_coords[1]), int(image_coords[0])), 10, 255, -1)
                    cv2.circle(image_depth, (int(depth_coords[1]), int(depth_coords[0])), 5, 255, -1)
                '''if count == 5:
                    break'''
        #resize and adjust for visualisation
        #image_color = cv2.resize(image_color, (0,0), fx=0.5, fy=0.5)
        #image_depth *= 1.0/10.0 # scale for visualisation (max range 10.0 m)
        cv2.imshow("image depth", image_depth)
        cv2.imshow("image color", image_color)
        cv2.waitKey(1)
        #print("centres", centres)
        #print("Len centres", len(centres))
        #print("Map coorinate", map_coordinate)
        #print("\nCount =\n", count)

        global way_point, image_count, g_nan_count    
        if way_point == "Take_Image":
            file1 = "grape"+ str(image_count)+".jpeg"
            file2 = "Depth_grape"+ str(image_count)+".jpeg"
            file3 = "Dilation_grape"+ str(image_count)+".jpeg"
            file4 = "grape_bunch"+ str(image_count)+".jpeg"
            grapes_coordinates.append(map_coordinate)
            g_nan_count = nan_count
            print("g_nan_count", g_nan_count)
            print("nan count", nan_count)
            cv2.imwrite(file1, image_color)
            cv2.imwrite(file2, image_depth)
            cv2.imwrite(file3, img2_updated)
            cv2.imwrite(file4, grape_bunch)
            print("Map_coordinate")
            way_point = "stop"
            image_count = image_count + 1
            print(map_coordinate)
            for key, value in map_coordinate.items():
                print(key, value)
            print("Count", count)
    
    def update_coordinate(self, state):
        global way_point
        way_point = state
        print("Updated")

    def get_grapes_coordinates(self):
        return grapes_coordinates 

    def get_nan_count(self):
        global g_nan_count
        print("get nan count", g_nan_count)
        return g_nan_count

'''def main(args):
    #Initializes and cleanup ros node
    rospy.init_node('image_projection', anonymous=True)
    ic = image_projection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)'''
