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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__VELOCITY_FEASIBILITY_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__VELOCITY_FEASIBILITY_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
/**
 * @file velocity_feasibility_cost_function.hpp
 * @brief Declaration of the VelocityFeasibilityCost class for computing velocity feasibility cost.
 *
 * This file contains the declaration of the VelocityFeasibilityCost class which computes the
 * cost based on the squared differences between the linear and angular velocities of two subsequent states.
 * The cost is applied only if the current position is within the control horizon. Otherwise, the cost is zero.
 */

/// @brief Namespace for the social MPC controller.
namespace nav2_social_mpc_controller
{

/**
 * @brief Cost function for enforcing velocity feasibility in the MPC controller.
 *
 * The VelocityFeasibilityCost class calculates the squared difference between the linear 
 * and angular velocities of two states. It scales the computed difference by a weight factor.
 * The computation is active only when the current position is less than the control horizon,
 * ensuring that the cost is applied appropriately during the control period.
 */
class VelocityFeasibilityCost
{
public:
  using VelocityFeasibilityCostFunction =
    ceres::AutoDiffCostFunction<VelocityFeasibilityCost, 1, 2, 2>;
  /**
   * @brief Constructs a new VelocityFeasibilityCost object.
   *
   * @param weight The cost weight applied to the squared differences.
   * @param current_position The current index position in the control horizon.
   * @param control_horizon The total number of positions (or time steps) over which control is applied.
   */
  VelocityFeasibilityCost(
    double weight, unsigned int current_position, unsigned int control_horizon);

  /**
    * @brief Creates a Ceres cost function for the VelocityFeasibilityCost.
    *
    * This function is a factory method that constructs an instance of the
    * VelocityFeasibilityCost, which is a Ceres cost function used to compute
    * the velocity feasibility cost between two states.
    *
    * @return A pointer to the created VelocityFeasibilityCost instance.
    */
  inline static VelocityFeasibilityCostFunction * Create(
    double weight, unsigned int current_position, unsigned int control_horizon)
  {
    return new VelocityFeasibilityCostFunction(
      new VelocityFeasibilityCost(weight, current_position, control_horizon));
  }
  /**
   * @brief Computes the velocity feasibility cost between two states.
   *
   * This templated operator computes the residual (cost) if the current position is within the control horizon.
   * It calculates the squared difference for subsequent linear and angular velocities weighted by a pre-defined factor.
   * If the current position is equal to or exceeds the control horizon, the residual is set to zero.
   *
   * @tparam T The arithmetic type (e.g., double) used for automatic differentiation.
   * @param state1 Pointer to the first state vector; state1[0] represents linear velocity and state1[1] angular velocity.
   * @param state2 Pointer to the second state vector with the same structure as state1.
   * @param residual Pointer to the residual value where the computed cost is stored.
   * @return true Always returns true indicating the computation was successful.
   */
  template <typename T>
  bool operator()(const T * const state1, const T * const state2, T * residual) const
  {
    if (current_position_ < control_horizon_) {
      auto lin_vel_diff = state1[0] - state2[0];
      auto ang_vel_diff = state1[1] - state2[1];
      residual[0] =
        (T)weight_ * (lin_vel_diff) * (lin_vel_diff) + (T)weight_ * (ang_vel_diff) * (ang_vel_diff);
    } else {
      residual[0] = (T)0.0;
    }
    return true;
  }

private:
  // Member variables:
  double weight_;                  ///< Weight factor applied to the cost computation.
  unsigned int control_horizon_;   ///< Total number of positions/time steps in the control horizon.
  unsigned int current_position_;  ///< The current position index within the control horizon.
};

}  // namespace nav2_social_mpc_controller

#endif