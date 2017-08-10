#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;



// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// For use in constraints of initial actuator value only
size_t delta_start_constraint = delta_start;
size_t a_start_constraint = delta_start + 1;


// Evaluate a polynomial.
AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

double polyeval2(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


// Evaluate the derivative a polynomial.
AD<double> polyderiveval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * CppAD::pow(x, i-1);
  }
  return result;
}



class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    double ref_v = ref_v_max;
    double y_at_50 = abs(polyeval2(coeffs, 50));    
    if (y_at_50 > max_y_fast)
    {
      if (y_at_50 > min_y_slow)
      {
        ref_v = ref_v_min;
      }
      else
      {
        ref_v = ref_v_min + (ref_v_max - ref_v_min) * (min_y_slow - y_at_50) / (min_y_slow - max_y_fast);
      }
    }

    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    
    // Cost to be optimized
    // In following calculations, multiplication factors have been chosen based on experimentations
    // and monitoring of individual cost contributions.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2) * pow(alpha, t);
      fg[0] += 10 * CppAD::pow(vars[epsi_start + t], 2) * pow(alpha, t);
      fg[0] += 0.03 * CppAD::pow(vars[v_start + t] - ref_v, 2) * pow(alpha, t);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 15 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2) * pow(alpha, t);
      fg[0] += 5 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2) * pow(alpha, t);
    }


    //
    // Setup Constraints

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    fg[1 + delta_start_constraint] = vars[delta_start];
    fg[1 + a_start_constraint] = vars[a_start];

    // The rest of the constraints
    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1] * delta_to_steering;
      AD<double> a0 = vars[a_start + t - 1] * throttle_to_acc;  // Ensures we convert throttle to acceleration

      // We have a polynomial
      AD<double> f0 = polyeval(coeffs, x0);
      AD<double> psides0 = CppAD::atan(polyderiveval(coeffs, x0)); // Based on derivative

      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt); // opposite sign due to simulator delta convention
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

void MPC::updateCoeffs(Eigen::VectorXd new_coeffs)
{
    coeffs = new_coeffs;
}

vector<double> MPC::Solve(Eigen::VectorXd state) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Extract data from state
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double steering = state[4];
  double throttle = state[5];
  double cte = state[6];
  double epsi = state[7];

  // Number of model variables (includes both states and inputs)
  // N time steps for 6 variables and N - 1 actuations for 2 actuators
  size_t n_vars = N * 6 + (N - 1) * 2;;
  
  // Number of constraints: all variables + initial actuators during latency
  size_t n_constraints = N * 6 + 2;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Initial variables
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  vars[delta_start] = steering;
  vars[a_start] = throttle;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
  constraints_lowerbound[delta_start_constraint] = steering;
  constraints_lowerbound[a_start_constraint] = throttle;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  constraints_upperbound[delta_start_constraint] = steering;
  constraints_upperbound[a_start_constraint] = throttle;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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

  // Copy all results into a vector (delta, acceleration and trajectory)
  vector<double> result(2 * N + 2);
  result[0] = solution.x[delta_start + 1]; // the first value is the latency one
  result[1] = solution.x[a_start + 1];
  
  for (size_t i=0; i<N; ++i)
  {
    result[2 + x_start + i] = solution.x[x_start + i]; // The first value is current one
    result[2 + y_start + i] = solution.x[y_start + i];
  }


  // Monitoring of actuators and every cost contribution
  /*
  double cost_cte = 0, cost_epsi = 0, cost_v = 0, cost_delta = 0, cost_a = 0;
  // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      cost_cte += CppAD::pow(solution.x[cte_start + t], 2) * pow(alpha, t);
      cost_epsi += 10 * CppAD::pow(solution.x[epsi_start + t], 2) * pow(alpha, t);
      cost_v += 0.03 * CppAD::pow(solution.x[v_start + t] - ref_v, 2) * pow(alpha, t);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      cost_delta += 15 * CppAD::pow(solution.x[delta_start + t + 1] - solution.x[delta_start + t], 2) * pow(alpha, t);
      cost_a += 5 * CppAD::pow(solution.x[a_start + t + 1] - solution.x[a_start + t], 2) * pow(alpha, t);
    }

    cout << "Acceleration / Delta: " << result[1] << " " << result[0] << endl;
    cout << "Cost (cte, epsi, v, delta, a) : " << cost_cte << " " << cost_epsi << " " << cost_v << " " << cost_delta << " " << cost_a << std::endl;
    */

  return result;
}
