#pragma once

#include "mpc_base.h"

class POS_MPC : public MPCBase{
public:
	POS_MPC();

    std::vector<double> mpc_x;
    std::vector<double> mpc_y;
    std::vector<double> mpc_theta;

    void LoadParams(const std::map<std::string, double> &params) override;
	std::vector<double> Solve(const Eigen::VectorXd &state) override;
    std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts) override;
    void updateGoal(const Eigen::Vector3d& goalPose) override;

    void setPoly(const Eigen::MatrixX4d& poly);

protected:
    double _x_goal, _y_goal;
    Eigen::MatrixX4d poly;
};
