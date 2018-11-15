#include <ros/ros.h>
#include <vector>

#include "rosplan_action_interface/RPActionInterface.h"

#ifndef PRESS_BUTTON_CLIENT
#define PRESS_BUTTON_CLIENT

/**
 * This file defines an action interface created in tutorial 10.
 */
namespace KCL_rosplan {

	class RPTutorialInterface: public RPActionInterface
	{

	private:
		ros::NodeHandle n;
	public:

		/* constructor */
		RPTutorialInterface(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
#endif