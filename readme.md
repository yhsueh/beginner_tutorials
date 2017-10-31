## Build procedures:
1. Create a catkin workspace using command catkin_make in the workspace folder.

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. Create a package inside workspace src folder using command catkin_create_pkg beginning_tutorials std_msgs rospy roscpp.

4. Go back to parent directory and input command catkin_make again.

5. Add talker and listener cpp files into the source folder under beginner_tutorials package.

6. Modify CmakeLists.txt to include talker and listener cpp files. 

7. Include necessary dependencies like roscpp and std_msgs and beginner_tutorials_generate_messages_cpp.


## Run procedures:
1. Create three terminals. Make sure the package's workspace is included in the PATH variable.

2. Input roscore to establish a master.

3. Input 
'''
rosrun beginner_tutorials talker
'''
in a different terminal to create the talker node.

4. Input 
'''
rosrun beginner_tutorials listener
'''
in the final terminal to create the listener node.



