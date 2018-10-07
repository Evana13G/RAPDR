#include "ros/ros.h"
#include "action_primitive_variation/PushButton.h"

bool push_button_action(action_primitive_variation::PushButton::Request  &req,
         action_primitive_variation::PushButton::Response &res){

  res.success_bool = 0;

  ROS_INFO("Request: Button name = %s", req.button_name);
  ROS_INFO("Sending back response: [%ld]", (long int)res.success_bool);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "push_button_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("push_button", push_button_action);
  ROS_INFO("Ready to push button.");
  ros::spin();

  return 0;
}