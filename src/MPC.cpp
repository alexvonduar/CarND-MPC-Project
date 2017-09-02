#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
//size_t N = 25;
//double dt = 0.05;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
//const double Lf = 2.67;

const double V_REF = 30;

const size_t X_OFF = 0;
const size_t Y_OFF = X_OFF + N;
const size_t PSI_OFF = Y_OFF + N;
const size_t V_OFF = PSI_OFF + N;
const size_t CTE_OFF = V_OFF + N;
const size_t EPSI_OFF = CTE_OFF + N;
const size_t DELTA_OFF = EPSI_OFF + N;
const size_t ACC_OFF = DELTA_OFF + N - 1;

// Weights of costs
const int W_CTE = 10000;
const int W_EPSI = 10000;
const int W_V = 10000;
const int W_DELTA = 100000;
const int W_ACC = 100000;
const int W_DELTA_RATE = 10000;
const int W_JERK = 10000;

class FG_eval
{
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars)
  {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    // Accumulate weighted state error
    for (int t = 0; t < N; t++)
    {
      fg[0] += W_CTE * CppAD::pow(vars[CTE_OFF + t], 2);
      fg[0] += W_EPSI * CppAD::pow(vars[EPSI_OFF + t], 2);
      fg[0] += W_V * CppAD::pow(vars[V_OFF + t] - V_REF, 2);
    }

    // Accumulate weighted actuator error
    for (int t = 0; t < N - 1; t++)
    {
      fg[0] += W_DELTA * CppAD::pow(vars[DELTA_OFF + t], 2);
      fg[0] += W_ACC * CppAD::pow(vars[ACC_OFF + t], 2);
    }

    // Accumulate weighte actuator changes
    for (int t = 0; t < N - 2; t++)
    {
      fg[0] += W_DELTA_RATE * CppAD::pow(vars[DELTA_OFF + t + 1] - vars[DELTA_OFF + t], 2);
      fg[0] += W_JERK * CppAD::pow(vars[ACC_OFF + t + 1] - vars[ACC_OFF + t], 2);
    }

    fg[1 + X_OFF]    = vars[X_OFF];
    fg[1 + Y_OFF]    = vars[Y_OFF];
    fg[1 + PSI_OFF]  = vars[PSI_OFF];
    fg[1 + V_OFF]    = vars[V_OFF];
    fg[1 + CTE_OFF]  = vars[CTE_OFF];
    fg[1 + EPSI_OFF] = vars[EPSI_OFF];

    // The rest of the constraints
    for (int t = 1; t < N; t++)
    {
      // state: t+1
      AD<double> x1    = vars[X_OFF + t];
      AD<double> y1    = vars[Y_OFF + t];
      AD<double> psi1  = vars[PSI_OFF + t];
      AD<double> v1    = vars[V_OFF + t];
      AD<double> cte1  = vars[CTE_OFF + t];
      AD<double> epsi1 = vars[EPSI_OFF + t];

      // state t
      AD<double> x0    = vars[X_OFF + t - 1];
      AD<double> y0    = vars[Y_OFF + t - 1];
      AD<double> psi0  = vars[PSI_OFF + t - 1];
      AD<double> v0    = vars[V_OFF + t - 1];
      AD<double> cte0  = vars[CTE_OFF + t - 1];
      AD<double> epsi0 = vars[EPSI_OFF + t - 1];

      // actuator t
      AD<double> delta0   = vars[DELTA_OFF + t - 1];
      AD<double> a0       = vars[ACC_OFF + t - 1];
      AD<double> f0       = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      AD<double> psi_des0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      fg[1 + X_OFF + t]    = x1    - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + Y_OFF + t]    = y1    - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + PSI_OFF + t]  = psi1  - (psi0 - v0 * delta0 / Lf * dt);
      fg[1 + V_OFF + t]    = v1    - (v0 + a0 * dt);
      fg[1 + CTE_OFF + t]  = cte1  - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + EPSI_OFF + t] = epsi1 - ((psi0 - psi_des0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * NState + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * NState;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++)
  {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[X_OFF]    = x;
  vars[Y_OFF]    = y;
  vars[PSI_OFF]  = psi;
  vars[V_OFF]    = v;
  vars[CTE_OFF]  = cte;
  vars[EPSI_OFF] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < DELTA_OFF; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = DELTA_OFF; i < ACC_OFF; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = ACC_OFF; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
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
  constraints_lowerbound[X_OFF]    = x;
  constraints_lowerbound[Y_OFF]    = y;
  constraints_lowerbound[PSI_OFF]  = psi;
  constraints_lowerbound[V_OFF]    = v;
  constraints_lowerbound[CTE_OFF]  = cte;
  constraints_lowerbound[EPSI_OFF] = epsi;

  constraints_upperbound[X_OFF]    = x;
  constraints_upperbound[Y_OFF]    = y;
  constraints_upperbound[PSI_OFF]  = psi;
  constraints_upperbound[V_OFF]    = v;
  constraints_upperbound[CTE_OFF]  = cte;
  constraints_upperbound[EPSI_OFF] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
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
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  vector<double> result;
  result.push_back(solution.x[DELTA_OFF]);
  result.push_back(solution.x[ACC_OFF]);
  for (int i = 0; i < N; ++i)
  {
    result.push_back(solution.x[X_OFF + i]);
    result.push_back(solution.x[Y_OFF + i]);
  }
  return result;
}
