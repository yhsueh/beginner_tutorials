#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <boost/shared_ptr.hpp>
#include <beginner_tutorials/ChangeString.h>

std::shared_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, must_pass)
{
	ros::ServiceClient client = nh->serviceClient< beginner_tutorials::ChangeString
      > ("change_string");
  bool exists(client.waitForExistence(ros::Duration(1)));
  EXPECT_TRUE(exists);

  beginner_tutorials::ChangeString srv;
  srv.request.input = "HELLO_WORLD";
  client.call(srv);

  EXPECT_EQ(srv.request.input, srv.response.reply);
}


int main(int argc,
         char **argv)
{
  ros::init(argc, argv, "talker_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

