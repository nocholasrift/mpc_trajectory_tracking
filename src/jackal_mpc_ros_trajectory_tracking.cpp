#include <math.h>
#include <tf/tf.h>
#include <algorithm>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include "jackal_mpc_ros_trajectory_tracking.h"

#define JACKAL_WIDTH .3


JackalMPCROS::JackalMPCROS(ros::NodeHandle &nh){

	_estop = false;
	_is_init = false;
	_is_goal = false;
    _traj_reset = false;

	_mpc_type = MPC_TYPE_CTE;

	velMsg.linear.x = 0;
	velMsg.angular.z = 0;

	double freq;

	// Localization params
	nh.param("jackal_mpc_track/use_vicon", _use_vicon, false);

	// MPC params
	nh.param("jackal_mpc_track/vel_pub_freq", _vel_pub_freq, 20.0);
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
	nh.param("jackal_mpc_track/w_pos", _w_pos, 1.0);

	// pos_mpc cost function params
	nh.param("jackal_mpc_track/pos_mpc_w_pos", _pos_mpc_w_pos, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_angvel", _pos_mpc_w_angvel, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_vel", _pos_mpc_w_vel, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_angvel_d", _pos_mpc_w_angvel_d, 1.0);
	nh.param("jackal_mpc_track/pos_mpc_w_linvel_d", _pos_mpc_w_linvel_d, .5);
	nh.param("jackal_mpc_track/pos_mpc_max_linvel", _pos_mpc_max_linvel, 2.0);
	nh.param("jackal_mpc_track/pos_mpc_max_angvel", _pos_mpc_max_angvel, 3.0);

	// Constraint params
	nh.param("jackal_mpc_track/max_angvel", _max_angvel, 3.0);
	nh.param("jackal_mpc_track/max_linvel", _max_linvel, 2.0);
	nh.param("jackal_mpc_track/max_lina", _max_lina, 3.0);
	nh.param("jackal_mpc_track/max_anga", _max_anga, 2*M_PI);
	nh.param("jackal_mpc_track/bound_value", _bound_value, 1.0e3);

	// Goal params
	nh.param("jackal_mpc_track/x_goal", _x_goal, 0.0);
	nh.param("jackal_mpc_track/y_goal", _y_goal, 0.0);
	nh.param("jackal_mpc_track/goal_tolerance", _tol, 0.3);

	// Teleop params
	nh.param("jackal_mpc_track/teleop", _teleop, false);
	nh.param<std::string>("jackal_mpc_track/frame_id", _frame_id, "odom");

	_dt=1.0/freq;

	_mpc_params["DT"] = _dt;
	_mpc_params["STEPS"] = _mpc_steps;
	_mpc_params["W_V"] = _w_linvel;
	_mpc_params["W_ANGVEL"] = _w_angvel;
	_mpc_params["W_DA"] = _w_linvel_d;
	_mpc_params["W_DANGVEL"] = _w_angvel_d;
	_mpc_params["W_ETHETA"] = _w_etheta;
	_mpc_params["W_POS"] = _w_pos;
	_mpc_params["W_CTE"] = _w_cte;
	_mpc_params["LINVEL"] = _max_linvel;
	_mpc_params["ANGVEL"] = _max_angvel;
	_mpc_params["BOUND"] = _bound_value;
	_mpc_params["X_GOAL"] = _x_goal;
	_mpc_params["Y_GOAL"] = _y_goal;

	_pos_mpc_params["DT"] = .1;
	_pos_mpc_params["STEPS"] = _mpc_steps;
	_pos_mpc_params["W_V"] = _pos_mpc_w_vel;
	_pos_mpc_params["W_ANGVEL"] = _pos_mpc_w_angvel;
	_pos_mpc_params["W_DA"] = _pos_mpc_w_linvel_d;
	_pos_mpc_params["W_DANGVEL"] = _pos_mpc_w_angvel_d;
	_pos_mpc_params["LINVEL"] = _pos_mpc_max_linvel;
	_pos_mpc_params["ANGVEL"] = _pos_mpc_max_angvel;
	_pos_mpc_params["BOUND"] = _bound_value;
	_pos_mpc_params["X_GOAL"] = _x_goal;
	_pos_mpc_params["Y_GOAL"] = _y_goal;

	_mpc.LoadParams(_mpc_params);
	_pos_mpc.LoadParams(_pos_mpc_params);

	if (_use_vicon)
		_odomSub = nh.subscribe("/vicon/jackal4/jackal4", 1, &JackalMPCROS::viconcb, this);
	else
		_odomSub = nh.subscribe("/odometry/filtered", 1, &JackalMPCROS::odomcb, this);
		  
    _polySub = nh.subscribe("/recoveryPoly", 1, &JackalMPCROS::polycb, this);
	_goalSub = nh.subscribe("recoveryGoal", 1, &JackalMPCROS::goalcb, this);
    _trajSub = nh.subscribe("/reference_trajectory", 1, &JackalMPCROS::trajectorycb, this);
    _trajNoResetSub = nh.subscribe("/reference_trajectory_no_reset", 1, &JackalMPCROS::trajectoryNoResetcb, this);

	_timer = nh.createTimer(ros::Duration(_dt), &JackalMPCROS::controlLoop, this);
	_velPubTimer = nh.createTimer(ros::Duration(1./_vel_pub_freq), &JackalMPCROS::publishVel, this);

    _pathPub = nh.advertise<nav_msgs::Path>("/spline_path", 10);
	_velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	_trajPub = nh.advertise<nav_msgs::Path>("/mpc_prediction", 10);
    _actualPathPub = nh.advertise<nav_msgs::Path>("/actual_path", 0);
    _pointPub = nh.advertise<geometry_msgs::PointStamped>("traj_point", 0);
    _odomPub = nh.advertise<visualization_msgs::Marker>("robot_position", 10);
	_polyPub = nh.advertise<geometry_msgs::PolygonStamped>("/convex_free", 0);
	_polyPub2 = nh.advertise<geometry_msgs::PolygonStamped>("/convex_free2", 0);
    _refPub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/current_reference", 10);
	_goalReachedPub = nh.advertise<std_msgs::Bool>("/mpc_goal_reached", 10);

	_eStop_srv = nh.advertiseService("/eStop", &JackalMPCROS::eStopcb, this);
	_mode_srv = nh.advertiseService("/switch_mode", &JackalMPCROS::mode_switchcb, this);
}

bool JackalMPCROS::eStopcb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_WARN("E-STOP RECEIVED");

	// geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;
	// _velPub.publish(velMsg);

	trajectory.points.clear();

	_estop ^= true;

	return true;
}

bool JackalMPCROS::mode_switchcb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
	ROS_WARN("SWITCHING MODE");

	_mpc_type = (_mpc_type == MPC_TYPE_CTE) ? MPC_TYPE_POS: MPC_TYPE_CTE;

	if (_mpc_type == MPC_TYPE_CTE)
		ROS_WARN("SWITCHED TO CTE MODE");
	else
		ROS_WARN("SWITCHED TO POS MODE");

	// geometry_msgs::Twist velMsg;
	velMsg.linear.x = 0;
	velMsg.angular.z = 0;
	// _velPub.publish(velMsg);

	_is_at_goal = false;
	_is_goal = false;

	return true;
}

void JackalMPCROS::polycb(const geometry_msgs::PoseArray::ConstPtr& msg){
	
	Eigen::MatrixX4d currPoly;
	for(int i = 0; i < msg->poses.size(); ++i){
		geometry_msgs::Pose p = msg->poses[i];
		if (p.orientation.x == 0 && p.orientation.y == 0 && p.orientation.z == 0 && p.orientation.w == 0){
			if (currPoly.rows() > 0){
				_poly = currPoly;
				break;
			}
		} else {
			Eigen::Vector4d plane(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
			currPoly.conservativeResize(currPoly.rows()+1, 4);
			currPoly.row(currPoly.rows()-1) = plane;
		}
	}
}

void JackalMPCROS::publishVel(const ros::TimerEvent&){
	_velPub.publish(velMsg);
}

void JackalMPCROS::goalcb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	_x_goal = msg->pose.position.x;
	_y_goal = msg->pose.position.y;
	
	_is_goal = true;
	_is_at_goal = false;

	ROS_WARN("GOAL RECEIVED (%.2f, %.2f)", _x_goal, _y_goal);
}

void JackalMPCROS::viconcb(const geometry_msgs::TransformStamped::ConstPtr& msg){

	tf::Quaternion q(
	    msg->transform.rotation.x,
	    msg->transform.rotation.y,
	    msg->transform.rotation.z,
	    msg->transform.rotation.w
	);

	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	_odom = Eigen::VectorXd(3);

	_odom(XI) = msg->transform.translation.x;
	_odom(YI) = msg->transform.translation.y;
	_odom(THETAI) = yaw;

	_is_init = true;

	visualization_msgs::Marker marker;
	marker.header.frame_id = _frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "position";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = _odom(XI);
	marker.pose.position.y = _odom(YI);
	marker.pose.position.z = 0;
	marker.pose.orientation.x = msg->transform.rotation.x;
	marker.pose.orientation.y = msg->transform.rotation.y;
	marker.pose.orientation.z = msg->transform.rotation.z;
	marker.pose.orientation.w = msg->transform.rotation.w;
	marker.scale.x = 0.4;
	marker.scale.y = 0.2;
	marker.scale.z = 0.4;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 1.0;
	marker.color.a = 1.0;
	_odomPub.publish(marker);
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

	if(!_is_init){
		_is_init = true;
		ROS_INFO("tracker initialized");
	}
}

// TODO: Support appending trajectories
void JackalMPCROS::trajectorycb(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
    trajectory = *msg;
    _traj_reset = true;
	
    ROS_INFO("MPC received trajectory!");
}

void JackalMPCROS::trajectoryNoResetcb(const trajectory_msgs::JointTrajectory::ConstPtr& msg){
    trajectory = *msg;
    ROS_INFO("MPC received trajectory (no time reset)!");
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

void JackalMPCROS::cte_ctrl_loop(){
	static ros::Time start;

	if (!_is_init || _estop)
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
        
        // geometry_msgs::Twist velMsg;
        // If trajectory is done, stop
        if ( t > traj_duration){
			ROS_INFO("trajectory done!");
            velMsg.linear.x = 0;
            velMsg.angular.z = 0;
            // _velPub.publish(velMsg);
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

		// account for periodicity of heading
		if (fabs(etheta) > M_PI){
			int sign = etheta >= 0 ? 1 : -1;
			if (sign == 1)
				etheta = 2*M_PI - etheta;
			else
				etheta = 2*M_PI + etheta;

			etheta *= -sign;
		}

        // MPC state consists of pose, cross-track error, and error in heading
        Eigen::VectorXd state(5);
        state << _odom(0), _odom(1), _odom(2), cte, etheta;
        
        // mpc_results setup to only contain the next time-step's inputs
        mpc_results.clear();
        mpc_results = _mpc.Solve(state, wpts);

		trajectory_msgs::JointTrajectoryPoint p = evalTraj(t);
		// ROS_INFO("[%.2f] current ref is (%.2f, %.2f, %.2f)", t, p.positions[0], p.positions[1], ref_head);
		// ROS_INFO("[%.2f] current state is (%.2f, %.2f, %.2f)", t, _odom(0), _odom(1), _odom(2));
		ROS_INFO("[%.2f] etheta is %.2f", t, etheta);
        ROS_INFO("{vel_x = %.2f, ang_z = %.2f}", mpc_results[1], mpc_results[0]);
		// ROS_ERROR("cte is %.4f", cte);

		double angle_to_ref = atan2(_odom(1)-wpts(3,0), _odom(0)-wpts(0,0)) - _odom(2);
        velMsg.angular.z = limit(velMsg.angular.z, mpc_results[0], _max_anga);
        velMsg.linear.x = limit(velMsg.linear.x, mpc_results[1], _max_lina);
        // _velPub.publish(velMsg);

        publishReference();
        publishMPCTrajectory();
        publishActualPath();

        // publish reference point
        geometry_msgs::PointStamped pointMsg;
        pointMsg.header.stamp = ros::Time::now();
        pointMsg.header.frame_id = _frame_id;
		int ind = 0; //std::min(20, (int)wpts.size());
        pointMsg.point.x = wpts(0,ind);
        pointMsg.point.y = wpts(3,ind);
        _pointPub.publish(pointMsg);
    }
}

void JackalMPCROS::pos_ctrl_loop(){
	static ros::Time start;

	if (!_is_init || _estop || !_is_goal || _poly.rows() == 0)
		return;

	Eigen::Vector2d goal = Eigen::Vector2d(_x_goal, _y_goal);
	Eigen::MatrixXd wpts(1, 2);
	wpts << _x_goal, _y_goal;

	// geometry_msgs::Twist velMsg;
	// If trajectory at goal, stop
	if ((Eigen::Vector2d(_odom(0), _odom(1))-goal).norm() < 25e-2 && !_is_at_goal){
		ROS_WARN("reached goal!");
		_is_at_goal = true;
		velMsg.linear.x = 0;
		velMsg.angular.z = 0;
		// _velPub.publish(velMsg);

		std_msgs::Bool msg;
		msg.data = true;
		_goalReachedPub.publish(msg);
		return;
	}


	// MPC state consists of pose, cross-track error, and error in heading
	Eigen::VectorXd state(3);
	state << _odom(0), _odom(1), _odom(2);
	
	_pos_mpc.setPoly(_poly);

	// mpc_results setup to only contain the next time-step's inputs
	mpc_results.clear();
	mpc_results = _pos_mpc.Solve(state, wpts);

	ROS_INFO("{vel_x = %.2f, ang_z = %.2f}", mpc_results[1], mpc_results[0]);

	velMsg.angular.z = limit(velMsg.angular.z, mpc_results[0], _max_anga);
	velMsg.linear.x = limit(velMsg.linear.x, mpc_results[1], _max_lina);
	// _velPub.publish(velMsg);

	publishMPCTrajectory();
	publishActualPath();

}

void JackalMPCROS::controlLoop(const ros::TimerEvent&){
    
	if (_mpc_type == MPC_TYPE_CTE)
		cte_ctrl_loop();
	else
		pos_ctrl_loop();
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
    msg.header.frame_id = _frame_id;

    bool published = false;
    for (trajectory_msgs::JointTrajectoryPoint pt : trajectory.points){
		if(!published){
			published = true;
			_refPub.publish(pt);
		}
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = _frame_id;

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
    msg.header.frame_id = _frame_id;
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
	goal.header.frame_id = _frame_id;
	goal.pose.position.x = _x_goal;
	goal.pose.position.y = _y_goal;
	goal.pose.orientation.w = 1;

	nav_msgs::Path pathMsg;
	pathMsg.header.frame_id = _frame_id;
	pathMsg.header.stamp = ros::Time::now();
	int size = (_mpc_type == MPC_TYPE_CTE) ? _mpc.mpc_x.size() : _pos_mpc.mpc_x.size();
	for(int i = 0; i < size; i++){
		double x, y;
		if (_mpc_type == MPC_TYPE_CTE){
			x = _mpc.mpc_x[i];
			y = _mpc.mpc_y[i];
		} else{
			x = _pos_mpc.mpc_x[i];
			y = _pos_mpc.mpc_y[i];
		}

		geometry_msgs::PoseStamped tmp;
		tmp.header = pathMsg.header;
		tmp.pose.position.x = x;
		tmp.pose.position.y = y;
		tmp.pose.position.z = .1;
		tmp.pose.orientation.w = 1;
		pathMsg.poses.push_back(tmp);
	}

	_trajPub.publish(pathMsg);

}
