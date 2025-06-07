// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__VELOCITY_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__VELOCITY_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
/**
 * @brief Cost function functor for velocity tracking using Ceres Solver.
 *
 * This struct computes the weighted squared error between the desired linear velocity and a predicted velocity
 * extracted from the optimization parameters. It is designed to be used with the Ceres AutoDiffCostCost to
 * automatically differentiate the cost function.
 *
 * The functor considers:
 * - A weight applied to the velocity difference.
 * - A desired linear velocity that the robot should maintain.
 * - The current position index within the control horizon.
 * - The control horizon defining the number of control steps.
 * - A block length that denotes for how many time steps a parameter is kept constant.
 *
 * The operator() function computes the residual as follows:
 * - If the current position is within the control horizon, it calculates the difference between the desired
 *   velocity and the parameter at the current time step (indexed by current_position_ divided by block_length_),
 *   squares it, and multiplies by the weight.
 * - If the current position exceeds the control horizon, the residual is set to zero.
 *
 * @tparam T The numeric type used for automatic differentiation.
 *
 * @param parameters An array of pointers to parameter blocks. The specific block used is determined by
 *                   current_position_ divided by block_length_.
 * @param residuals  Output pointer where the computed residual is stored.
 *
 * @return true Always returns true to indicate that the cost computation was successful.
 */
namespace nav2_social_mpc_controller
{

class VelocityCost
{
public:
  using VelocityCostFunction = ceres::DynamicAutoDiffCostFunction<VelocityCost>;
  VelocityCost(
    double weight, double desired_linear_vel, unsigned int current_position,
    unsigned int control_horizon, unsigned int block_length);

  /**
  * @brief Creates a Ceres cost function for the VelocityCost.
  *
  * This function is a factory method that constructs an instance of the
  * VelocityCost, which is a Ceres cost function
  * for computing the velocity cost based on the difference between the desired
  * linear velocity and the predicted velocity from the optimization parameters.
  */
  inline static VelocityCostFunction * Create(
    double weight, double desired_linear_vel, unsigned int current_position,
    unsigned int control_horizon, unsigned int block_length)
  {
    return new VelocityCostFunction(new VelocityCost(
      weight, desired_linear_vel, current_position, control_horizon, block_length));
  }
  /**
  * @brief operator function for the velocity cost function.
  *
  * This function computes the residuals for the velocity cost function.
  * It calculates the difference between the desired linear velocity and the
  * current linear velocity, squares it, and multiplies by the weight.
  *
  * @tparam T The type of the parameters, typically double.
  * @param parameters An array of pointers to the parameters, where each pointer
  *                   points to a block of parameters for the current time step.
  * @param residuals An array where the computed residuals will be stored.
  * @return true if the computation was successful, false otherwise.
  */
  template <typename T>
  bool operator()(T const * const * parameters, T * residuals) const
  {
    if (current_position_ < control_horizon_) {
      auto linear_diff = (T)desired_linear_vel_ - parameters[current_position_ / block_length_][0];
      residuals[0] = (T)weight_ * (linear_diff) * (linear_diff);
    } else {
      residuals[0] = (T)0.0;
    }
    return true;
  }

private:
  double weight_;
  double desired_linear_vel_;
  unsigned int current_position_;
  unsigned int control_horizon_;
  unsigned int block_length_;
};

}  // namespace nav2_social_mpc_controller

#endif