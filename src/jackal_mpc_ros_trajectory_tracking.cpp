#include <math.h>
#include <tf/tf.h>
#include <algorithm>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "jackal_mpc_ros_trajectory_tracking.h"

#define JACKAL_WIDTH .3

JackalMPCROS::JackalMPCROS(ros::NodeHandle &nh){

	_is_init = false;
	_is_goal = false;
    _traj_reset = false;

	double freq;

	// MPC params
	nh.param("jackal_mpc_track/controller_frequency", freq, 10.0);
	nh.param("jackal_mpc_track/mpc_steps", _mpc_steps, 10.0);

	// Cost function params
	nh.param("jackal_mpc_track/w_vel",_w_vel,1.0);
	nh.param("jackal_mpc_track/w_angvel",_w_angvel,1.0);
	nh.param("jackal_mpc_track/w_linvel",_w_linvel,1.0);
	nh.param("jackal_mpc_track/w_angvel_d",_w_angvel_d,1.0);
	nh.param("jackal_mpc_track/w_linvel_d",_w_linvel_d,.5);
	nh.param("jackal_mpc_track/w_etheta", _w_etheta, 1.0);
	nh.param("jackal_mpc_track/w_cte", _w_cte, 1.0);

	// Constraint params
	nh.param("jackal_mpc_track/max_angvel", _max_angvel, 3.0);
	nh.param("jackal_mpc_track/max_linvel", _max_linvel, 2.0);
	nh.param("jackal_mpc_track/max_lina", _max_lina, 3.0);
	nh.param("jackal_mpc_track/max_anga", _max_anga, M_PI/2);
	nh.param("jackal_mpc_track/bound_value", _bound_value, 1.0e3);

	// Goal params
	nh.param("jackal_mpc_track/x_goal", _x_goal, 0.0);
	nh.param("jackal_mpc_track/y_goal", _y_goal, 0.0);
	nh.param("jackal_mpc_track/theta_goal", _theta_goal, 0.0);
	nh.param("jackal_mpc_track/goal_tolerance", _tol, 0.3);

	// Teleop params
	nh.param("jackal_mpc_track/teleop", _teleop, false);

	_dt=1.0/freq;

	_mpc_params["DT"] = _dt;
	_mpc_params["STEPS"] = _mpc_steps;
	_mpc_params["W_V"] = _w_linvel;
	_mpc_params["W_ANGVEL"] = _w_angvel;
	_mpc_params["W_DA"] = _w_linvel_d;
	_mpc_params["W_DANGVEL"] = _w_angvel_d;
	_mpc_params["W_ETHETA"] = _w_etheta;
	_mpc_params["W_CTE"] = _w_cte;
	_mpc_params["LINVEL"] = _max_linvel;
	_mpc_params["ANGVEL"] = _max_angvel;
	_mpc_params["BOUND"] = _bound_value;
	_mpc_params["X_GOAL"] = _x_goal;
	_mpc_params["Y_GOAL"] = _y_goal;
	_mpc_params["THETA_GOAL"] = _theta_goal;

	_mpc.LoadParams(_mpc_params);

	_goalSub = nh.subscribe("/gapGoal", 1, &JackalMPCROS::goalcb, this);
	_odomSub = nh.subscribe("/odometry/filtered", 1, &JackalMPCROS::odomcb, this);
    _trajSub = nh.subscribe("/reference_trajectory", 1, &JackalMPCROS::trajectorycb, this);
	_timer = nh.createTimer(ros::Duration(_dt), &JackalMPCROS::controlLoop, this);

    _pathPub = nh.advertise<nav_msgs::Path>("/spline_path", 10);
	_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	_trajPub = nh.advertise<nav_msgs::Path>("/mpc_prediction", 10);
    _pointPub = nh.advertise<geometry_msgs::PointStamped>("traj_point", 0);
	_polyPub = nh.advertise<geometry_msgs::PolygonStamped>("/convex_free", 0);
	_polyPub2 = nh.advertise<geometry_msgs::PolygonStamped>("/convex_free2", 0);
    _actualPathPub = nh.advertise<nav_msgs::Path>("/actual_path", 0);
}

void JackalMPCROS::goalcb(const std_msgs::Float32MultiArray::ConstPtr& msg){
	_mpc.updateGoal(Eigen::Vector3d(msg->data[0], msg->data[1], msg->data[2]));
	_x_goal = msg->data[0];
	_y_goal = msg->data[1];
	_theta_goal = msg->data[2];
	_is_goal = true;
}

void JackalMPCROS::odomcb(const nav_msgs::Odometry::ConstPtr& msg){

	tf::Quaternion q(
	    msg->pose.pose.orientation.x,
	    msg->pose.pose.orientation.y,
	    msg->pose.pose.orientation.z,
	    msg->pose.pose.orientation.w
	);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	_odom = Eigen::VectorXd(3);

	_odom(XI) = msg->pose.pose.position.x;
	_odom(YI) = msg->pose.pose.position.y; 
	_odom(THETAI) = yaw;

	_curr_vel = msg->twist.twist.linear.x;
	_curr_ang_vel = msg->twist.twist.angular.z;

	_is_init = true;
}

// TODO: Support appending trajectories
void JackalMPCROS::trajectorycb(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
    trajectory = *msg;
    _traj_reset = true;

    ROS_INFO("MPC received trajectory!");
}

double JackalMPCROS::limit(double prev_v, double input, double max_rate){
	double ret = 0.0;
	if (fabs(prev_v-input)/_dt > max_rate){

		if (input > prev_v)
			ret = prev_v + max_rate*_dt;
		else
			ret = prev_v - max_rate*_dt;
	}
	else
		ret = input;

	return ret;
}

void JackalMPCROS::controlLoop(const ros::TimerEvent&){
    static ros::Time start;

	if (!_is_init)
		return;

    if (_traj_reset){
        start = ros::Time::now();
        _traj_reset = false;
    }

    if (trajectory.points.size() != 0){

        ros::Time now = ros::Time::now();
        double t = (now-start).toSec();

        double traj_duration = trajectory.points.back().time_from_start.toSec();
		Eigen::Vector2d goal(trajectory.points.back().positions[0],
							 trajectory.points.back().positions[1]);
        
        geometry_msgs::Twist velMsg;
        // If trajectory is done, stop
        if ( t > traj_duration){
            velMsg.linear.x = 0;
            velMsg.angular.z = 0;
            _velPub.publish(velMsg);
            return;
        }

		Eigen::Vector2d odom_se2(_odom(0), _odom(1));
		if ((goal-odom_se2).squaredNorm() < .1){
			velMsg.linear.x = 0;
			velMsg.angular.z = 0;
			_velPub.publish(velMsg);
			return;
		}

        // Send next _mpc_steps reference points to solver
        // For feasible tracking, trajectory MUST be C2-continuous due to
        // trajectory angular velocity calculations in MPC
        Eigen::MatrixXd wpts(6, (int)_mpc_steps);
        for(int i = 0; i < _mpc_steps; i++){
            trajectory_msgs::JointTrajectoryPoint pt;

            if (i*_dt + t > traj_duration)
                pt = trajectory.points.back();
            else
                pt = evalTraj(i*_dt + t);

            // posX, velX, accX
            wpts(0,i) = pt.positions[0];
            wpts(1,i) = i*_dt + t >= traj_duration ? 0: pt.velocities[0];
            wpts(2,i) = pt.accelerations[0];

            // posY, velY, accY
            wpts(3,i) = pt.positions[1];
            wpts(4,i) = i*_dt + t >= traj_duration ? 0: pt.velocities[1];
            wpts(5,i) = pt.accelerations[1];

        }

        // compute CTE and ETHETA
        // Found equations for these values at:
        // https://ntnuopen.ntnu.no/ntnu-xmlui/bitstream/handle/11250/280167/FossenPettersenGaleazzi2014.pdf?sequence=3
        
        double ref_head = atan2(wpts(4,0), wpts(1,0));
        double cte = -1*(_odom(0)-wpts(0,0))*sin(ref_head) + (_odom(1)-wpts(3,0))*cos(ref_head);
        double etheta = _odom(2) - ref_head;

		// ROS_INFO("odom: (%.2f, %.2f)\tref: (%.2f, %.2f, %.2f)", _odom(0), _odom(1), wpts(0,0), wpts(3,0), ref_head);
		// ROS_INFO("cte: %.2f\tetheta: %.2f", cte, etheta);
        // MPC state consists of pose, cross-track error, and error in heading
        Eigen::VectorXd state(5);
        state << _odom(0), _odom(1), _odom(2), cte, etheta;
        
        // mpc_results setup to only contain the next time-step's inputs
        mpc_results.clear();
        mpc_results = _mpc.Solve(state, wpts);
        ROS_INFO("{vel_x = %.2f, ang_z = %.2f}", mpc_results[1], mpc_results[0]);

        velMsg.angular.z = mpc_results[0];
        velMsg.linear.x = mpc_results[1];
        _velPub.publish(velMsg);

        publishReference();
        // put this after so MPC prediction is overtop path in RVIZ
        publishMPCTrajectory();
        publishActualPath();

        // publish reference point
        geometry_msgs::PointStamped pointMsg;
        pointMsg.header.stamp = ros::Time::now();
        pointMsg.header.frame_id = "odom";
        pointMsg.point.x = wpts(0,0);
        pointMsg.point.y = wpts(3,0);
        _pointPub.publish(pointMsg);
    }

}

trajectory_msgs::JointTrajectoryPoint JackalMPCROS::evalTraj(double t){
    
    for (trajectory_msgs::JointTrajectoryPoint pt : trajectory.points){
        if (t < pt.time_from_start.toSec())
            return pt;
    }

    return trajectory.points.back();

}

void JackalMPCROS::publishReference(){

    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";

    for (trajectory_msgs::JointTrajectoryPoint pt : trajectory.points){
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "odom";

        pose.pose.position.x = pt.positions[0];
        pose.pose.position.y = pt.positions[1];
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        msg.poses.push_back(pose);
    }

    _pathPub.publish(msg);
}


void JackalMPCROS::publishActualPath(){

    // for visualizing actual path
    poses.push_back(_odom);
        
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom";
    for(Eigen::Vector3d p : poses){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p[0];
        pose.pose.position.y = p[1];
        pose.pose.position.z = 0;

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        msg.poses.push_back(pose);
    }
    
    _actualPathPub.publish(msg);
}


void JackalMPCROS::publishMPCTrajectory(){

	geometry_msgs::PoseStamped goal;
	goal.header.stamp = ros::Time::now();
	goal.header.frame_id = "odom";
	goal.pose.position.x = _x_goal;
	goal.pose.position.y = _y_goal;
	goal.pose.orientation.w = 1;

	nav_msgs::Path pathMsg;
	pathMsg.header.frame_id = "odom";
	pathMsg.header.stamp = ros::Time::now();
	for(int i = 0; i < _mpc.mpc_x.size(); i++){
		geometry_msgs::PoseStamped tmp;
		tmp.header = pathMsg.header;
		tmp.pose.position.x = _mpc.mpc_x[i];
		tmp.pose.position.y = _mpc.mpc_y[i];
		tmp.pose.orientation.w = 1;
		pathMsg.poses.push_back(tmp);
	}

	_trajPub.publish(pathMsg);

}
