# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp rospy std_msgs genmsg message_generation)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv ChangeString.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp beginner_tutorials_gencpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(change_string_client src/change_string_client.cpp)
target_link_libraries(change_string_client ${catkin_LIBRARIES})
add_dependencies(change_string_client beginner_tutorials_generate_messages_cpp beginner_tutorials_gencpp)

# %EndTag(FULLTEXT)%
