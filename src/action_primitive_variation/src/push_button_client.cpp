#include "ros/ros.h"
#include "action_primitive_variation/PushButton.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "push_two_buttons_client");
  if (argc != 2)
  {
    ROS_INFO("usage: push_button_client X ");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<action_primitive_variation::PushButton>("push_button");
  action_primitive_variation::PushButton srv;
  srv.request.button_name = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("Success: %ld", (long int)srv.response.success_bool);
  }
  else
  {
    ROS_ERROR("Failed to call service push_button");
    return 1;
  }

  return 0;
}