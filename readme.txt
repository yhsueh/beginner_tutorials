Build and Run procedures
1. Create a catkin workspace using command catkin_make in the workspace folder

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. Create a package inside workspace src folder using command catkin_create_pkg beginning_tutorials std_msgs rospy roscpp

4. Go back to parent directory and input command catkin_make again.

5.

