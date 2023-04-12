#ifndef JACKAL_MPC_ROS_TRAJECTORY_TRACKING_H
#define JACKAL_MPC_ROS_TRAJECTORY_TRACKING_H

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "jackal_mpc_trajectory_tracking.h"

class JackalMPCROS {
public:

	JackalMPCROS(ros::NodeHandle &nh);

	void LoadParams(const std::map<std::string, double> &params);

private:

	ros::NodeHandle _nh;
	ros::Subscriber _odomSub, _laserSub, _trajSub, _trajNoResetSub;
	ros::Publisher _velPub, _trajPub, _polyPub, _polyPub2,
     _pathPub, _pointPub, _actualPathPub, _odomPub;
	ros::Timer _timer;

	Eigen::VectorXd _odom;

    trajectory_msgs::JointTrajectory trajectory;

    std::vector<Eigen::Vector3d> poses;
	std::vector<double> mpc_results;

	MPC _mpc;
    std::map<std::string, double> _mpc_params;
    double _mpc_steps, _w_vel, _w_angvel, _w_linvel, _w_angvel_d, _w_linvel_d, _w_etheta,
    	_max_angvel, _max_linvel, _bound_value, _x_goal, _y_goal, _theta_goal, _tol,
    	_max_lina, _max_anga, _w_cte, _w_pos;

    const int XI = 0;
    const int YI = 1;
    const int THETAI = 2;

    double _dt, _curr_vel, _curr_ang_vel;
    bool _is_init, _is_goal, _teleop, _traj_reset, _use_vicon;

	std::string _frame_id;

	void publishMPCTrajectory();
    void publishActualPath();
    void publishReference();

    trajectory_msgs::JointTrajectoryPoint evalTraj(double t);

	void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
	void goalcb(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void viconcb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void trajectorycb(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
	void trajectoryNoResetcb(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

	void controlLoop(const ros::TimerEvent&);
	double limit(double prev_v, double input, double max_rate);
};

#endif
