#ifndef JACKAL_MPC_TRAJECTORY_TRACKING_H
#define JACKAL_MPC_TRAJECTORY_TRACKING_H

#include <vector>
#include <Eigen/Core>


class MPC{
public:
	MPC();

    std::vector<double> mpc_x;
    std::vector<double> mpc_y;
    std::vector<double> mpc_theta;

    void LoadParams(const std::map<std::string, double> &params);
	std::vector<double> Solve(const Eigen::VectorXd &state);
    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts);
    void updateGoal(const Eigen::Vector3d& goalPose);

private:
	// Parameters for mpc solver
    double _max_angvel, _max_linvel, _bound_value, _x_goal, _y_goal, _theta_goal;
    int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, 
    _angvel_start, _linvel_start, _w_start, _w_pos;
    std::map<std::string, double> _params;

    unsigned int dis_cnt;
};

#endif
