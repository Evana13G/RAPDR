#include "rosplan_action_interface/PressButtonClient.h"
#include "geometry_msgs/PoseStamped.h"
#include "action_primitive_variation/PressButtonSrv.h"
#include "action_primitive_variation/CloseGripperSrv.h"
#include <iostream>

/* The implementation of RPTutorial.h */
namespace KCL_rosplan {

	/* constructor */
	RPTutorialInterface::RPTutorialInterface(ros::NodeHandle &nh) {
		n = nh;
	}

	/* action dispatch callback */
	bool RPTutorialInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// The action implementation goes here.
		printf("Concrete Callback section reached");

		// ros::ServiceClient client = n.serviceClient<action_primitive_variation::PressButtonSrv>("PressButtonSrv");
		// action_primitive_variation::PressButtonSrv srv;
		// srv.request.limb = "left_gripper";
  // 		srv.request.buttonName = "button1";
		// // srv.request.buttonPoseStamped = NULL; 
		// ros::service::waitForService("PressButtonSrv");
		// if (client.call(srv)){
		// 	ROS_INFO("Success: %ld", (long int)srv.response.success_bool);
		// }else{
		// 	ROS_ERROR("Failed to call service");
		// 	return 1;
		// }


		ros::ServiceClient client = n.serviceClient<action_primitive_variation::CloseGripperSrv>("CloseGripperSrv");
		action_primitive_variation::CloseGripperSrv srv;
		srv.request.limb = "left_gripper";
		// srv.request.buttonPoseStamped = NULL; 
		ros::service::waitForService("CloseGripperSrv");
		if (client.call(srv)){
			ROS_INFO("Success: %ld", (long int)srv.response.success_bool);
		}else{
			ROS_ERROR("Failed to call service");
			return 1;
		}


		// complete the action
		ROS_INFO("KCL: (%s) TUTORIAL Action completing.", msg->name.c_str());
		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "press_button_client", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPTutorialInterface rpti(nh);

		rpti.runActionInterface();

		// The code never comes back to main

		return 0;
	}
