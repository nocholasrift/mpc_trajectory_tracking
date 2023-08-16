#pragma once

#include <map>
#include <vector>
#include <Eigen/Core>

class MPCBase {
public:

    std::vector<double> mpc_x;
    std::vector<double> mpc_y;
    std::vector<double> mpc_theta;

    virtual void LoadParams(const std::map<std::string, double> &params) = 0;
    virtual std::vector<double> Solve(const Eigen::VectorXd &state) = 0;
    virtual std::vector<double> Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts) = 0;
    virtual void updateGoal(const Eigen::Vector3d& goalPose) = 0;

protected:

    double _max_angvel, _max_linvel, _bound_value;
    int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _angvel_start, 
    _linvel_start, _w_start, _w_pos;
    std::map<std::string, double> _params;
};
