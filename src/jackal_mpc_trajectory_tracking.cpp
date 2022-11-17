#include <string>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <cppad/ipopt/solve.hpp>
#include "jackal_mpc_trajectory_tracking.h"

// ROS IPOPT-based MPC for differential drive/skid-steering vehicles
// Code modifies the car-based MPC tracking github below:
// https://github.com/Danziger/CarND-T2-P5-Model-Predictive-Control-MPC

namespace {
     using CppAD::AD;

     class FG_eval {
     public:

     	double _dt, _ref_cte, _ref_etheta, _ref_vel, _x_goal, _y_goal, _theta_goal; 
        double  _w_cte, _w_etheta, _w_vel, _w_angvel, _w_angvel_d, _w_linvel_d, _w_pos;
        int _mpc_steps;

        AD<double> cost_cte, cost_etheta, cost_vel;

        FG_eval() { 

        	// Set default value    
            _dt = 0.1;  // in sec
            _ref_cte   = 0;
            _ref_etheta  = 0;
            _ref_vel   = 0.5; // m/s

            _x_goal = 0.0;
            _y_goal = 0.0;
            _theta_goal = 0.0;

            // Cost function weights
            _w_cte     = 100;
            _w_etheta    = .5;
            _w_vel     = 0;
            _w_angvel   = 0;
            _w_angvel_d = 0;
            _w_linvel_d = 0;
            _w_pos = 1;

            _mpc_steps   = 10;
            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
            // _v_start     = _theta_start + _mpc_steps;
            _cte_start = _theta_start + _mpc_steps;
            _etheta_start = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _linvel_start     = _angvel_start + _mpc_steps - 1;
        }

        // Load parameters for constraints
        void LoadParams(const std::map<std::string, double> &params)
        {
            _dt = params.find("DT") != params.end() ? params.at("DT") : _dt;
            _mpc_steps = params.find("STEPS") != params.end()    ? params.at("STEPS") : _mpc_steps;
            _ref_cte   = params.find("REF_CTE") != params.end()  ? params.at("REF_CTE") : _ref_cte;
            _ref_etheta  = params.find("REF_ETHETA") != params.end() ? params.at("REF_ETHETA") : _ref_etheta;
            _ref_vel   = params.find("REF_V") != params.end()    ? params.at("REF_V") : _ref_vel;
            
            _w_cte   = params.find("W_CTE") != params.end()   ? params.at("W_CTE") : _w_cte;
            _w_etheta  = params.find("W_ETHETA") != params.end()  ? params.at("W_ETHETA") : _w_etheta;
            _w_vel   = params.find("W_V") != params.end()     ? params.at("W_V") : _w_vel;
            _w_angvel = params.find("W_ANGVEL") != params.end() ? params.at("W_ANGVEL") : _w_angvel;
            _w_angvel_d = params.find("W_DANGVEL") != params.end() ? params.at("W_DANGVEL") : _w_angvel_d;
            _w_linvel_d = params.find("W_DA") != params.end()     ? params.at("W_DA") : _w_linvel_d;

            _x_goal = params.find("X_GOAL") != params.end() ? params.at("X_GOAL") : _x_goal;
            _y_goal = params.find("Y_GOAL") != params.end() ? params.at("Y_GOAL") : _y_goal;
            _theta_goal = params.find("THETA_GOAL") != params.end() ? params.at("THETA_GOAL") : _theta_goal;

            if (_theta_goal < 0)
            	_theta_goal += 2*M_PI;

            _x_start     = 0;
            _y_start     = _x_start + _mpc_steps;
            _theta_start   = _y_start + _mpc_steps;
		    // _v_start     = _theta_start + _mpc_steps;
            _cte_start = _theta_start + _mpc_steps;
            _etheta_start = _cte_start + _mpc_steps;
            _angvel_start = _etheta_start + _mpc_steps;
            _linvel_start     = _angvel_start + _mpc_steps - 1;
            
            //cout << "\n!! FG_eval Obj parameters updated !! " << _mpc_steps << endl; 
        }

        void setWpts(const Eigen::MatrixXd &wpts){
            this->wpts = wpts;
        }

        void setGoal(const Eigen::Vector3d& goalPose){
            _x_goal = goalPose(0);
            _y_goal = goalPose(1);
            _theta_goal = goalPose(2);
        }

        // MPC implementation (cost func & constraints)
        typedef CPPAD_TESTVECTOR(AD<double>) ADvector; 
        void operator()(ADvector& fg, const ADvector& vars) {
            // cost function is fg[0]
            fg[0] = 0;

            // ROS_INFO("GOAL: (%.2f, %.2f)", _x_goal, _y_goal);

            // for (int i = 0; i < _mpc_steps; i++){
            // 	fg[0] += _w_pos*CppAD::pow(vars[_x_start+i]-_x_goal,2);
            // 	fg[0] += _w_pos*CppAD::pow(vars[_y_start+i]-_y_goal,2);
            // }
            for(int i = 0; i < _mpc_steps; i++){

                AD<double> ref_vel_x = wpts(1,i);
                AD<double> ref_vel_y = wpts(4,i);
                AD<double> ref_vel = CppAD::sqrt(CppAD::pow(ref_vel_x,2)+CppAD::pow(ref_vel_y,2));

                fg[0] += _w_cte * CppAD::pow(vars[_cte_start+i], 2);
                fg[0] += _w_etheta * CppAD::pow(vars[_etheta_start+i], 2);
                fg[0] += _w_vel * CppAD::pow(vars[_linvel_start+i]-ref_vel, 2);
            }

            // Minimize the use of actuators.
            for (int i = 0; i < _mpc_steps - 1; i++) {
              fg[0] += _w_angvel * CppAD::pow(vars[_angvel_start + i],2);
            //   fg[0] += _w_vel * CppAD::pow(vars[_linvel_start + i],2);
            }

            // // Minimize the value gap between sequential actuations.
            for (int i = 0; i < _mpc_steps - 2; i++) {
              fg[0] += _w_angvel_d * CppAD::pow(vars[_angvel_start + i + 1] - vars[_angvel_start + i], 2);
              fg[0] += _w_linvel_d * CppAD::pow(vars[_linvel_start + i + 1] - vars[_linvel_start + i], 2);
            }

            // fg[x] for constraints
            // Initial constraints
            fg[1 + _x_start] = vars[_x_start];
            fg[1 + _y_start] = vars[_y_start];
            fg[1 + _theta_start] = vars[_theta_start];
            fg[1 + _cte_start] = vars[_cte_start];
            fg[1 + _etheta_start] = vars[_etheta_start];

            // Add system dynamic model constraint
            for (int i = 0; i < _mpc_steps - 1; i++)
            {
                // The state at time t+1 .
                AD<double> x1 = vars[_x_start + i + 1];
                AD<double> y1 = vars[_y_start + i + 1];
                AD<double> theta1 = vars[_theta_start + i + 1];
                AD<double> cte1 = vars[_cte_start + i + 1];
                AD<double> etheta1 = vars[_etheta_start + i + 1];

                // The state at time t.
                AD<double> x0 = vars[_x_start + i];
                AD<double> y0 = vars[_y_start + i];
                AD<double> theta0 = vars[_theta_start + i];
                AD<double> cte0 = vars[_cte_start + i];
                AD<double> etheta0 = vars[_etheta_start + i];

                // trajectory angular vel (omega)
                AD<double> ref_vel_x = wpts(1,i);
                AD<double> ref_acc_x = wpts(2,i);
                AD<double> ref_vel_y = wpts(4,i);
                AD<double> ref_acc_y = wpts(5,i);

                AD<double> traj_omg;
                if (ref_vel_x*ref_vel_x + ref_vel_y*ref_vel_y < 1e-6)
                    traj_omg = 0;
                else{
                    traj_omg = (ref_acc_y*ref_vel_x - ref_acc_x*ref_vel_y)/
                    (CppAD::pow(ref_vel_x,2) + CppAD::pow(ref_vel_y,2));
                }

                // Only consider the actuation at time t.
                //AD<double> angvel0 = vars[_angvel_start + i];
                AD<double> w0 = vars[_angvel_start + i];
                AD<double> v0 = vars[_linvel_start + i];

                // Here's `x` to get you started.
                // The idea here is to constraint this value to be 0.
                //
                // NOTE: The use of `AD<double>` and use of `CppAD`!
                // This is also CppAD can compute derivatives and pass
                // these to the solver.

                if (fabs(w0) > 1e-8){
                	fg[2 + _x_start + i] = x1 - (x0 + v0/w0 * (CppAD::sin(theta0+_dt*w0)-CppAD::sin(theta0)));
                	fg[2 + _y_start + i] = y1 - (y0 + v0/w0 * (CppAD::cos(theta0)-CppAD::cos(theta0+_dt*w0)));
                	fg[2 + _theta_start + i] = theta1 - (theta0 +  w0 * _dt);
                } else{
                	fg[2 + _x_start + i] = x1 - (x0 + v0 * CppAD::cos(theta0) * _dt);
                	fg[2 + _y_start + i] = y1 - (y0 + v0 * CppAD::sin(theta0) * _dt);
                	fg[2 + _theta_start + i] = theta1 - (theta0 +  w0 * _dt);
                }

                fg[2 + _cte_start + i] = cte1 - (cte0 + v0 * CppAD::sin(etheta0) * _dt);
                fg[2 + _etheta_start + i] = etheta1 - (etheta0 + (w0 - traj_omg) * _dt); 
            }


        }

    private:
    	int _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _linvel_start;
        Eigen::MatrixXd wpts;
    };
}


MPC::MPC(){
	// Set default value    
    _mpc_steps = 10;
    _max_angvel = 3.0; // Maximal angvel radian (~30 deg)
    _max_linvel = 2.0; // Maximal linvel accel
    _bound_value  = 1.0e3; // Bound value for other variables

    // Goal defaults
    _x_goal = 0.0;
    _y_goal = 0.0;
    _theta_goal = 0.0;

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    // _v_start     = _theta_start + _mpc_steps;
    _cte_start = _theta_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _linvel_start     = _angvel_start + _mpc_steps - 1;
}

void MPC::LoadParams(const std::map<std::string, double> &params)
{
    _params = params;
    //Init parameters for MPC object
    _mpc_steps = _params.find("STEPS") != _params.end() ? _params.at("STEPS") : _mpc_steps;
    _max_angvel = _params.find("ANGVEL") != _params.end() ? _params.at("ANGVEL") : _max_angvel;
    _max_linvel = _params.find("LINVEL") != _params.end() ? _params.at("LINVEL") : _max_linvel;
    _bound_value  = _params.find("BOUND") != _params.end()  ? _params.at("BOUND") : _bound_value;
    
    _x_goal = params.find("X_GOAL") != params.end() ? params.at("X_GOAL") : _x_goal;
    _y_goal = params.find("Y_GOAL") != params.end() ? params.at("Y_GOAL") : _y_goal;
    _theta_goal = params.find("THETA_GOAL") != params.end() ? params.at("THETA_GOAL") : _theta_goal;

    if (_theta_goal < 0)
    	_theta_goal += 2*M_PI;

    _x_start     = 0;
    _y_start     = _x_start + _mpc_steps;
    _theta_start   = _y_start + _mpc_steps;
    // _v_start     = _theta_start + _mpc_steps;
    _cte_start = _theta_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _linvel_start     = _angvel_start + _mpc_steps - 1;

    std::cout << "\n!! MPC Obj parameters updated !! " << std::endl; 
}


std::vector<double> MPC::Solve(const Eigen::VectorXd &state) 
{
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    // const double v = state[3];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * state.size() + (_mpc_steps - 1) * (state.size()-1);
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * state.size();

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    // vars[_v_start] = v;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // for (int i = _v_start; i < _angvel_start; i++){
    //     vars_lowerbound[i] = -_max_linvel;
    //     vars_upperbound[i] = _max_linvel;
    // }

    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _linvel_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }

    // Acceleration/decceleration upper and lower limits
    for (int i = _linvel_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_linvel;
        vars_upperbound[i] = _max_linvel;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    // constraints_lowerbound[_v_start] = v;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    // constraints_upperbound[_v_start] = v;


    // object that computes objective and constraints
    FG_eval fg_eval;
    fg_eval.LoadParams(_params);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          .5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    // std::cout << "starting solver" << std::endl;
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success
    		|| solution.status == CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point
    		|| solution.status == CppAD::ipopt::solve_result<Dvector>::feasible_point_found);

    if (!ok){
    	std::cout << "Error occured during solve (" << solution.status << ")" << std::endl;
    	solution.x[_angvel_start] = 0;
    	solution.x[_linvel_start] = 0;
    }

    // Cost
    auto cost = solution.obj_value;
    // std::cout << "------------ Total Cost(solution): " << cost << " ------------" << std::endl;
    // std::cout << "max_angvel:" << _max_angvel <<std::endl;
    // std::cout << "max_linvel:" << _max_linvel <<std::endl;
 
    // std::cout << "-----------------------------------------------" <<std::endl;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);

        // std::cout << "(" << solution.x[_x_start+i] << ", " << solution.x[_y_start+i] << ", " << solution.x[_theta_start+i] << ")" << std::endl;
    }

    std::vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_linvel_start]);

    return result;
}

std::vector<double> MPC::Solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &wpts) 
{
    bool ok = true;

    typedef CPPAD_TESTVECTOR(double) Dvector;
    const double x = state[0];
    const double y = state[1];
    const double theta = state[2];
    const double cte = state[3];
    const double etheta = state[4];
    // const double v = state[3];

    // Set the number of model variables (includes both states and inputs).
    // For example: If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    size_t n_vars = _mpc_steps * state.size() + (_mpc_steps - 1) * (state.size()-1);
    
    // Set the number of constraints
    size_t n_constraints = _mpc_steps * state.size();

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    // Set the initial variable values
    vars[_x_start] = x;
    vars[_y_start] = y;
    vars[_theta_start] = theta;
    vars[_cte_start] = cte;
    vars[_etheta_start] = etheta;
    // vars[_v_start] = v;

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _angvel_start; i++) 
    {
        vars_lowerbound[i] = -_bound_value;
        vars_upperbound[i] = _bound_value;
    }
    // for (int i = _v_start; i < _angvel_start; i++){
    //     vars_lowerbound[i] = -_max_linvel;
    //     vars_upperbound[i] = _max_linvel;
    // }

    // The upper and lower limits of angvel are set to -25 and 25
    // degrees (values in radians).
    for (int i = _angvel_start; i < _linvel_start; i++) 
    {
        vars_lowerbound[i] = -_max_angvel;
        vars_upperbound[i] = _max_angvel;
    }

    // Acceleration/decceleration upper and lower limits
    for (int i = _linvel_start; i < n_vars; i++)  
    {
        vars_lowerbound[i] = -_max_linvel;
        vars_upperbound[i] = _max_linvel;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    constraints_lowerbound[_x_start] = x;
    constraints_lowerbound[_y_start] = y;
    constraints_lowerbound[_theta_start] = theta;
    constraints_lowerbound[_cte_start] = cte;
    constraints_lowerbound[_etheta_start] = etheta;
    // constraints_lowerbound[_v_start] = v;
    constraints_upperbound[_x_start] = x;
    constraints_upperbound[_y_start] = y;
    constraints_upperbound[_theta_start] = theta;
    constraints_upperbound[_cte_start] = cte;
    constraints_upperbound[_etheta_start] = etheta;
    // constraints_upperbound[_v_start] = v;


    // object that computes objective and constraints
    FG_eval fg_eval;
    fg_eval.LoadParams(_params);
    fg_eval.setWpts(wpts);

    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  0\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          .5\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    // std::cout << "starting solver" << std::endl;
    CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= (solution.status == CppAD::ipopt::solve_result<Dvector>::success
    		|| solution.status == CppAD::ipopt::solve_result<Dvector>::stop_at_acceptable_point
    		|| solution.status == CppAD::ipopt::solve_result<Dvector>::feasible_point_found);

    if (!ok){
    	std::cout << "Error occured during solve (" << solution.status << ")" << std::endl;
    	solution.x[_angvel_start] = 0;
    	solution.x[_linvel_start] = 0;
    }

    // Cost
    auto cost = solution.obj_value;
    // std::cout << "------------ Total Cost(solution): " << cost << " ------------" << std::endl;
    // std::cout << "max_angvel:" << _max_angvel <<std::endl;
    // std::cout << "max_linvel:" << _max_linvel <<std::endl;
 
    // std::cout << "-----------------------------------------------" <<std::endl;

    this->mpc_x = {};
    this->mpc_y = {};
    this->mpc_theta = {};
    for (int i = 0; i < _mpc_steps; i++) 
    {
        this->mpc_x.push_back(solution.x[_x_start + i]);
        this->mpc_y.push_back(solution.x[_y_start + i]);
        this->mpc_theta.push_back(solution.x[_theta_start + i]);

        // std::cout << "(" << solution.x[_x_start+i] << ", " << solution.x[_y_start+i] << ", " << solution.x[_theta_start+i] << ")" << std::endl;
    }

    std::vector<double> result;
    result.push_back(solution.x[_angvel_start]);
    result.push_back(solution.x[_linvel_start]);

    return result;
}


void MPC::updateGoal(const Eigen::Vector3d& goalPose){
    _x_goal = goalPose(0);
    _y_goal = goalPose(1);
    _theta_goal = goalPose(2);

    _params["X_GOAL"] = _x_goal;
    _params["Y_GOAL"] = _y_goal;
    _params["THETA_GOAL"] = _theta_goal;

}

double limit(double prev_v, double input){
	double ret = 0.0;
	if (fabs(prev_v-input)/.1 > 3){

		if (input > prev_v)
			ret = prev_v + 3*.1;
		else
			ret = prev_v - 3*.1;
	}
	else
		ret = input;

	return ret;
}
