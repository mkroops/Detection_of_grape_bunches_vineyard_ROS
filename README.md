The robot moves autonomously near to vineyard coarse by using topological navigation. Intially it moves to waypoint 0 and end with waypoint 6.
so it is completely covered the vineyard. It captures images and stores the grapes bunches cooardinates and remove duplicate bunches and estimate
overall grape bunches count. This system converts 2d coordinates of image to 3d coordinates in map to identify where the grape bunches is located.
it has 7 nodes. where main node follow state machine to execute the tasks properly and keep on track.

The following state machine are

-START = 0 #start 
-MOVE_TO_WAY_POINT = 1 #move to all waypoints which is mentioned below
TAKE_IMAGE = 2 #take image
IMAGE_TAKEN = 3 #update imgae has taken
ROTATE_TO_VINEYARD = 4 #rotate to vineyard
ESTIMATE_GRAPE_BUNCHES = 5 #estimate total grape bunches
STOP = 6 #stop and exit

It stores recording in bagfile folder and images in images folder.

How to execute
1.create workspace and execute commands in terminal
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

2. Insert robot_programming_project in src folder

3.update .bashrc file and source the package.
cd ~
vi .bashrc
source ~/catkin_ws/devel/setup.bash
catkin_make

4. To avoid unwanted logs inbetween folow the command below
roslaunch bacchus_gazebo vineyard_demo.launch world_name:=vineyard_small
roslaunch robot_programming_project grape_bunch_counter.launch

or
To execute in a single line follow the below command
roslaunch robot_programming_project grape_bunch_counter_with_bacchus_gazebo.launch


