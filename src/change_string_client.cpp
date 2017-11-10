/*
 * Copyright (C) 2017, Yuyu Hsueh.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstdlib>
#include "beginner_tutorials/ChangeString.h"
#include "ros/ros.h"

/** This is the client node of the service "change_string". One would enter a string argument, which would
 * be sent to the server node. And then, the server node would change its publishing content based on the 
 * string provided by the user. If no service is called, then the default string is "Hello".
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "change_string_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient < beginner_tutorials::ChangeString
      > ("change_string");
  beginner_tutorials::ChangeString srv;
  
  if (argc < 2) {
    ROS_FATAL("usage: Error! Please enter a desired string");
    return 1;
  }

  if (argc > 2) {
    ROS_WARN("More than one string is provided, only the first is taken");
  }

  srv.request.input = argv[1];

  if (client.call(srv)) {
    ROS_DEBUG("Successfully call service change_string");
    ROS_INFO("The string has been changed to %s", srv.response.reply.c_str());
  } else {
    ROS_ERROR("Failed to call service change_string");
    return 1;
  }

  return 0;
}
