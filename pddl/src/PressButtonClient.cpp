#include "rosplan_action_interface/PressButtonClient.h"
#include "geometry_msgs/PoseStamped.h"
#include "agent/PressButtonSrv.h"
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
		ROS_INFO("Concrete Callback section reached");

		// ros::ServiceClient client = n.serviceClient<agent::PressButtonSrv>("PressButtonSrv");
		// agent::PressButtonSrv srv;
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


		ros::ServiceClient client = n.serviceClient<agent::PressButtonSrv>("/press_button_srv");
		agent::PressButtonSrv srv;
		srv.request.limb = "left";
		srv.request.buttonName = "button1";
		// srv.request.buttonPoseStamped = NULL; 
		ros::service::waitForService("press_button_srv");
		if (client.call(srv)){
			ROS_INFO("Success: %ld", (long int)srv.response.success_bool);
		}else{
			ROS_ERROR("Failed to call service");
			return 1;
		}

		// complete the action
		ROS_INFO("KCL: (%s) Press Button action completing.", msg->name.c_str());
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
