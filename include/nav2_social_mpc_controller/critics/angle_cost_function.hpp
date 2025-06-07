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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__ANGLE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__ANGLE_COST_FUNCTION_HPP_

#include <nav2_social_mpc_controller/update_state.hpp>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
/**
 * @brief Cost function for orienting a robot towards a target point.
 *
 * This cost function computes the angular error between the direction from the updated robot state
 * and the direction towards a given target point. The updated state is computed using the current robot 
 * pose and control parameters over a defined time step and control horizon. The computed error is then 
 * scaled by a configurable weight, and its squared value forms the residual used in the optimization process.
 *
 * Use this function with Ceres Solver's autodifferentiation framework to iteratively adjust control inputs 
 * and drive the robot's state to align its heading with the target direction.
 *
 * @param weight A scaling factor to adjust the influence of the angle constraint.
 * @param point A 2D point (Eigen::Matrix<double, 2, 1>) representing the target toward which the heading is computed.
 * @param robot_init The initial pose of the robot (geometry_msgs::msg::Pose) used as the starting point for state updates.
 * @param current_position The current position index in the control sequence.
 * @param time_step The duration between control updates.
 * @param control_horizon The total number of control steps over which the state is updated.
 * @param block_length The length of the parameter block used in the optimization problem, typically related to the number of control inputs.
 *
 * @tparam T The type used for automatic differentiation. It is typically set to double or a Jet type provided by Ceres.
 *
 * @return Always returns true to indicate a successful evaluation.
 */
namespace nav2_social_mpc_controller
{

class AngleCost
{
public:
  using AngleCostFunction = ceres::DynamicAutoDiffCostFunction<AngleCost>;
  AngleCost(
    double weight, const Eigen::Matrix<double, 2, 1> point,
    const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
    unsigned int control_horizon, unsigned int block_length);

  /**
    * @brief Creates a Ceres cost function for the AngleCost.
    *
    * This function is a factory method that constructs an instance of the
    * AngleCost using the provided parameters.
    *
    * @param weight The weight for the cost function.
    * @param point The target point towards which the robot should align its heading.
    * @param robot_init The initial pose of the robot.
    * @param current_position The current position in the planning sequence.
    * @param time_step The time step for the MPC.
    * @param control_horizon The total number of time steps in the MPC.
    * @param block_length The length of the parameter block for the MPC.
    * @return A pointer to the created AutoDiffCostFunction instance.
    */
  inline static AngleCostFunction * Create(
    double weight, const Eigen::Matrix<double, 2, 1> point,
    const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
    unsigned int control_horizon, unsigned int block_length)
  {
    return new AngleCostFunction(new AngleCost(
      weight, point, robot_init, current_position, time_step, control_horizon, block_length));
  }

  /**
   * @brief Operator to compute the angle cost based on the robot's state and target point.
   *
   * This function computes the angular error between the direction from the updated robot state
   * and the direction towards a given target point. The updated state is computed using the current robot pose
   * and control parameters over a defined time step and control horizon.
   *
   * @param parameters Double pointer to robot state parameters.
   * @param residuals Pointer to the computed residual used in the optimization problem.
   * @return True if successful, false otherwise.
   */
  template <typename T>
  bool operator()(T const * const * parameters, T * residuals) const
  {
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
      robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);
    auto dx = point_[0] - new_position_x;
    auto dy = point_[1] - new_position_y;
    auto point_heading = atan2(dy, dx);
    auto new_yaw = new_position_orientation;
    new_yaw = atan2(sin(new_yaw), cos(new_yaw));
    auto angle_diff = (T)point_heading - new_yaw;
    residuals[0] = (T)weight_ * (angle_diff) * (angle_diff);

    return true;
  }

private:
  double weight_;
  const Eigen::Matrix<double, 2, 1> point_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
};

}  // namespace nav2_social_mpc_controller

#endif