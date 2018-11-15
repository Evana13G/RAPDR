#include "rosplan_action_interface/ObtainObjectClient.h"
#include "geometry_msgs/PoseStamped.h"
#include "action_primitive_variation/ObtainObjectSrv.h"
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

		ros::ServiceClient client = n.serviceClient<action_primitive_variation::ObtainObjectSrv>("/obtain_object_srv");
		action_primitive_variation::ObtainObjectSrv srv;
		srv.request.limb = "left";
		srv.request.objectName = "object";
		// srv.request.buttonPoseStamped = NULL; 
		ros::service::waitForService("obtain_object_srv");
		if (client.call(srv)){
			ROS_INFO("Success: %ld", (long int)srv.response.success_bool);
		}else{
			ROS_ERROR("Failed to call service");
			return 1;
		}

		// complete the action
		ROS_INFO("KCL: (%s) Obtain Object action completing.", msg->name.c_str());
		return true;
	}
} // close namespace

	/*-------------*/
	/* Main method */
	/*-------------*/

	int main(int argc, char **argv) {

		ros::init(argc, argv, "obtain_object_client", ros::init_options::AnonymousName);
		ros::NodeHandle nh("~");

		// create PDDL action subscriber
		KCL_rosplan::RPTutorialInterface rpti(nh);

		rpti.runActionInterface();

		// The code never comes back to main

		return 0;
	}
