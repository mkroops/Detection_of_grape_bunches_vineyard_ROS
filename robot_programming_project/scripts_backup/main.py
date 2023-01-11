import sys, rospy
from time import sleep
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from image_projection import image_projection
from set_topo_nav_goal import set_topo_nav_goal
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math 

roll = pitch = yaw = 0.0
same_image = "no"
#https://www.theconstructsim.com/ros-qa-135-how-to-rotate-a-robot-to-a-desired-heading-using-feedback-from-odometry/

def odom(odom_data):
    global roll, pitch, yaw
    robot_position = odom_data.pose.pose
    quaternion = (robot_position.orientation.x, robot_position.orientation.y, robot_position.orientation.z, robot_position.orientation.w)
    (roll, pitch, yaw) = euler_from_quaternion(quaternion)

def rotate(target_angle):
    kP = 0.5
    max_angle = math.pi / 180
    t = Twist()
    r = rospy.Rate(10)
    target_rad = target_angle * math.pi/180
    
    publisher = rospy.Publisher(
            '/thorvald_001/teleop_joy/cmd_vel',
            Twist, queue_size=1)
    while True:
        if math.fabs(target_rad - yaw) > max_angle:
            t.angular.z = kP  * (target_rad - yaw)
            publisher.publish(t)
        else:
            return

def remove_duplicates(grapes_coordinates, img_1, img_2, tolerence_level):
    img_duplicate_cnt = 0
    for i in range(len(grapes_coordinates[img_1])):
        image_1 = grapes_coordinates[img_1]
        img_1_max_boundry_x =  image_1[i][0] + tolerence_level
        img_1_min_boundry_x =  image_1[i][0] - tolerence_level
        img_1_max_boundry_y =  image_1[i][1] + tolerence_level
        img_1_min_boundry_y =  image_1[i][1] - tolerence_level
        img_1_max_boundry_z =  image_1[i][2] + tolerence_level
        img_1_min_boundry_z =  image_1[i][2] - tolerence_level
        print("###################")
        print(i)
        print("###################")
        #print(image_1[i], image_4[j])
        flag = False
        for j in range(len(grapes_coordinates[img_2])):
            image_2 = grapes_coordinates[img_2]
            if (same_image == "yes") and (i==j):
                continue
            if ( (img_1_min_boundry_z <= image_2[j][2] <= img_1_max_boundry_z) and 
                (img_1_min_boundry_y <= image_2[j][1] <= img_1_max_boundry_y) and
                (img_1_min_boundry_x <= image_2[j][0] <= img_1_max_boundry_x)):
                    print("image", img_1, i, j)
                    print(image_1[i], image_2[j])
                    flag = True
        if flag == True:
            img_duplicate_cnt =  img_duplicate_cnt + 1
    print("Image 1 4 count", img_duplicate_cnt)
    return img_duplicate_cnt

def main(args):
    '''Initializes and cleanup ros node'''
    print("done")
    ic = image_projection()
    print("done")
    topo_nav_goal = set_topo_nav_goal()
    rospy.Subscriber('/thorvald_001/odometry/base_raw', Odometry, odom)

    topo_nav_goal.move_to_topological_node("WayPoint0")
    sleep(10)
    print("waiting 10s")
    ic.update_coordinate("Take_Image")
    nan_count_1 = ic.get_nan_count()
    sleep(5)

    topo_nav_goal.move_to_topological_node("WayPoint1")
    sleep(10)
    print("waiting 10s")
    ic.update_coordinate("Take_Image")
    nan_count_2 = ic.get_nan_count()
    sleep(5)

    topo_nav_goal.move_to_topological_node("WayPoint2")
    topo_nav_goal.move_to_topological_node("WayPoint3")
    print("reached waypoint 3")
    
    topo_nav_goal.move_to_topological_node("WayPoint4")
    rotate(180)
    sleep(10)
    print("waiting 10s")
    ic.update_coordinate("Take_Image")
    nan_count_3 = ic.get_nan_count()
    sleep(5)

    topo_nav_goal.move_to_topological_node("WayPoint5")
    rotate(180)
    sleep(10)
    print("waiting 10s")
    ic.update_coordinate("Take_Image")
    nan_count_4 = ic.get_nan_count()
    sleep(5)

    topo_nav_goal.move_to_topological_node("WayPoint6")
    topo_nav_goal.move_to_topological_node("WayPoint7")
    
    grapes_coordinates = ic.get_grapes_coordinates()
    for map_coordinate in grapes_coordinates:
        for key, value in map_coordinate.items():
            print(key, value)
    
    image_1_count = len(grapes_coordinates[0])
    image_2_count = len(grapes_coordinates[1])
    image_3_count = len(grapes_coordinates[2])
    image_4_count = len(grapes_coordinates[3])
    global same_image
    same_image = "yes"
    duplicate_count_img_1_1 = int((remove_duplicates(grapes_coordinates, 0, 0, 0.15)) / 2)
    duplicate_count_img_2_2 = int((remove_duplicates(grapes_coordinates, 1, 1, 0.15)) / 2)
    duplicate_count_img_3_3 = int((remove_duplicates(grapes_coordinates, 2, 2, 0.15)) / 2)
    duplicate_count_img_4_4 = int((remove_duplicates(grapes_coordinates, 3, 3, 0.15)) / 2)
    same_image = "no"
    duplicate_count_img_1_4 = remove_duplicates(grapes_coordinates, 0, 3, 0.5)
    duplicate_count_img_2_3 = remove_duplicates(grapes_coordinates, 1, 2, 0.5)
    cnt_img_1_4 = (image_1_count + image_4_count) - duplicate_count_img_1_4
    cnt_img_2_3 = (image_2_count + image_3_count) - duplicate_count_img_2_3
    max_nan_1 = nan_count_1 if nan_count_1 > nan_count_4 else nan_count_4
    max_nan_2 = nan_count_2 if nan_count_2 > nan_count_3 else nan_count_3
    overall_count = (cnt_img_1_4 + cnt_img_2_3) - ( duplicate_count_img_1_1 + duplicate_count_img_2_2 
                    + duplicate_count_img_3_3 + duplicate_count_img_4_4) + (max_nan_1 + max_nan_2)
    print("Image 1 count =", image_1_count)
    print("Image 2 count =", image_2_count)
    print("Image 3 count =", image_3_count)
    print("Image 4 count =", image_4_count)
    print("Duplicate count Image 1, 4 =", duplicate_count_img_1_4)
    print("Duplicate count Image 2, 3 =", duplicate_count_img_2_3)
    print("Count 1, 4 =", cnt_img_1_4)
    print("Count 2, 3 =", cnt_img_2_3)
    print("Duplicate Count same image 1 2 3 4", duplicate_count_img_1_1, 
                      duplicate_count_img_2_2, duplicate_count_img_3_3, duplicate_count_img_4_4)
    print("Nan Count 1,2,3,4,14,23", nan_count_1,nan_count_2, nan_count_3, nan_count_4, max_nan_1,max_nan_2)
    print("Overall Grape Count =", overall_count)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)