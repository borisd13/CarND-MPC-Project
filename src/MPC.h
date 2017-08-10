#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// Model variables

const int polynomial_order = 3;  // Polynomial order to approximate waypoints
const double alpha = 1.1; // Muliplication factor to give more importance to future than present
                           // It limits noise of movement and allow to correct position slowly

// We base our prediction on 1 seconds to anticipate turns which seems to be enough
const size_t N = 10;   // Number of steps
const double dt = 0.1; // this matches the latency time, which simplifies the model

// Actuator parameters
const double throttle_to_acc = 1.8;      // Convert throttle to acceleration, based on experimentations
const double delta_to_steering = 0.85;  // Convert delta angle to steering, based on experimentations
const double ref_v_max = 120, ref_v_min = 65;   // Target speed, varies based on proximity to curves
const double max_y_fast = 7, min_y_slow = 14;   // Parameters to evaluate whether we're close to a curve and how sharp it is

// Length from front to CoG that has a similar radius.
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
// Lf was tuned until the the radius formed by MPC model matched the previous radius.
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;


class MPC {
 public:
  MPC();
  Eigen::VectorXd coeffs;
  virtual ~MPC();

  // Update coeffs with new estimate
  void updateCoeffs(Eigen::VectorXd);

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state);
};

#endif /* MPC_H */
