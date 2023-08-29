#ifndef JACKAL_MPC_ROS_TRAJECTORY_TRACKING_H
#define JACKAL_MPC_ROS_TRAJECTORY_TRACKING_H

#include <string>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "jackal_mpc_pos.h"
#include "jackal_mpc_trajectory_tracking.h"

enum mpc_type
{
	MPC_TYPE_CTE = 0,
	MPC_TYPE_POS = 1
};

class JackalMPCROS {
public:

	JackalMPCROS(ros::NodeHandle &nh);

	void LoadParams(const std::map<std::string, double> &params);

private:

	ros::NodeHandle _nh;
	
	ros::Subscriber _odomSub, _goalSub, _trajSub, _trajNoResetSub, _polySub;

	ros::Publisher _velPub, _trajPub, _polyPub, _polyPub2,
     _pathPub, _pointPub, _actualPathPub, _odomPub, _refPub,
	 _goalReachedPub;

	ros::ServiceServer _eStop_srv, _mode_srv;

	ros::Timer _timer, _velPubTimer;

	Eigen::VectorXd _odom;

    trajectory_msgs::JointTrajectory trajectory;

    std::vector<Eigen::Vector3d> poses;
	std::vector<double> mpc_results;

	MPC _mpc;
	POS_MPC _pos_mpc;
    std::map<std::string, double> _mpc_params;
    std::map<std::string, double> _pos_mpc_params;
    double _mpc_steps, _w_vel, _w_angvel, _w_linvel, _w_angvel_d, _w_linvel_d, _w_etheta,
    	_max_angvel, _max_linvel, _bound_value, _x_goal, _y_goal, _theta_goal, _tol,
    	_max_lina, _max_anga, _w_cte, _w_pos;

	double _pos_mpc_w_pos, _pos_mpc_w_angvel, _pos_mpc_w_angvel_d, _pos_mpc_w_linvel_d, 
		_pos_mpc_w_vel, _pos_mpc_max_linvel, _pos_mpc_max_angvel;

    const int XI = 0;
    const int YI = 1;
    const int THETAI = 2;

    double _dt, _curr_vel, _curr_ang_vel, _vel_pub_freq;
    bool _is_init, _is_goal, _teleop, _traj_reset, _use_vicon, _estop,
		_is_at_goal;

	Eigen::MatrixX4d _poly;
	geometry_msgs::Twist velMsg;

	std::string _frame_id;

	void publishMPCTrajectory();
    void publishActualPath();
    void publishReference();

    trajectory_msgs::JointTrajectoryPoint evalTraj(double t);

	void cte_ctrl_loop();
	void pos_ctrl_loop();

	void odomcb(const nav_msgs::Odometry::ConstPtr& msg);
	void polycb(const geometry_msgs::PoseArray::ConstPtr& msg);
	void goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void viconcb(const geometry_msgs::TransformStamped::ConstPtr& msg);
    void trajectorycb(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
	void trajectoryNoResetcb(const trajectory_msgs::JointTrajectory::ConstPtr& msg);
	
	// services
	bool eStopcb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
	bool mode_switchcb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	void publishVel(const ros::TimerEvent&);
	void controlLoop(const ros::TimerEvent&);
	double limit(double prev_v, double input, double max_rate);

	mpc_type _mpc_type;
};

#endif
