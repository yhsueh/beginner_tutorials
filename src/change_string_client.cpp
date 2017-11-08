#include "ros/ros.h"
#include "beginner_tutorials/ChangeString.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "change_string_client");
  if (argc != 2)
  {
    ROS_INFO("usage: Error! Please enter a desired string");
    return 1;
  }

    ROS_INFO("HI");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::ChangeString>("change_string");
  beginner_tutorials::ChangeString srv;

  srv.request.input = argv[1];


  if (client.call(srv))
  {
    ROS_INFO("Successfully call service change_string");
    ROS_INFO("The string has been changed to %s",srv.response.reply.c_str());
  }
  else
  {
    ROS_ERROR("Failed to call service change_string");
    return 1;
  }

  return 0;
}