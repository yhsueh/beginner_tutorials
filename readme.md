## Build procedures:
1. Clone Week10_HW branch from github by inputting
```
git clone -b Week10_HW https://github.com/yhsueh/beginner_tutorials.git
```

2. Move beginner_tutorials into src folder in your catkin_ws.

2. Make sure ROS_PACKAGE_PATH enviroment variable contain the workspace folder. This is done by sourcing the generated setup file under devel folder in the parent directory.

3. Input 
```
catkin_make
```
to build the ROS package.

## Procedures for using service to change talker's string:
1. Create four terminals. Make sure the package's workspace is included in the PATH variable.

2. Input roscore to establish a master.

3. Input 
```
rosrun beginner_tutorials talker
```
in a different terminal to create the talker node.

4. Input 
```
rosrun beginner_tutorials listener
```
in the final terminal to create the listener node.

5. Input
```
rosrun beginner_tutorials change_string_client <your-string-without bracket>
```
If no argument is passed, a fatal message will be returned. If more than one string is provided, only the first string is passed.

## Procedures for using roslaunch to adjust publishing rate:
1. Similar to last section. Create four terminals and repeat the procedures until the forth step.

2. Input
```
roslaunch beginner_tutorials node.launch frequency:=<an integer>
```
If no argument is passed, a error message will be returned and set frequency to one. The displaying rate would increase or decrease based on the provided frequency.

3. rqt_console and rqt_logger_level can display ROS logs. To use them, create two terminals and input the following:
```
rosrun rqt_console rqt_console
```
and
```
rosrun rqt_logger_level rqt_logger_level
```
