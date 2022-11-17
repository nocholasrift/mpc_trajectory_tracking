#include <ros/ros.h>
#include "jackal_mpc_ros_trajectory_tracking.h"

int main(int argc, char **argv){

	ros::init(argc, argv, "jackal_mpc_planner");
	ros::NodeHandle nh;

	JackalMPCROS jackalMPC(nh);

	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
