# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

## Compiler Version
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
tf roscpp rospy rostest std_msgs genmsg message_generation)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv ChangeString.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)

add_executable(talker src/talker.cpp src/passCustStr.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp beginner_tutorials_gencpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(change_string_client src/change_string_client.cpp)
target_link_libraries(change_string_client ${catkin_LIBRARIES})
add_dependencies(change_string_client beginner_tutorials_generate_messages_cpp beginner_tutorials_gencpp)

if(CATKIN_ENABLE_TESTING)
add_rostest_gtest(talker_test
  test/talker_test.test
  test/talker_test.cpp
)
target_link_libraries(talker_test ${catkin_LIBRARIES})
endif()
# %EndTag(FULLTEXT)%
