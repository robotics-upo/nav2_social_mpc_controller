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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__GOAL_ALIGN_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__GOAL_ALIGN_COST_FUNCTION_HPP_
#include "Eigen/Core"
#include "angles/angles.h"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "nav2_social_mpc_controller/update_state.hpp"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"

/**
 * @brief Cost functor for aligning the robot's heading with a desired goal heading.
 *
 * This struct defines a cost function intended for use with Ceres Solver to
 * penalize the deviation between the robot’s heading and a target goal heading.
 * The cost is computed as the squared turning angle difference, weighted by a user-defined factor.
 *
 * The operator() function propagates the robot state based on a sequence of control
 * inputs using the function computeUpdatedStateRedux, then calculates the turning angle
 * between the computed new heading and the desired goal heading (given in goal_heading_).
 * The residual output is produced by squaring the turning angle difference and multiplying
 * it by the weight.
 *
 * @note The function computeUpdatedStateRedux is assumed to provide the updated state,
 *       returning new x-position, y-position, and orientation after applying the control inputs.
 *
 * @param weight_         Weight factor applied to the cost.
 * @param goal_heading_   Desired goal heading represented as a 2x1 Eigen matrix.
 * @param robot_init_     Initial pose of the robot.
 * @param current_position_ The current position index from where control updates are applied.
 * @param time_step_      Time step duration for the state propagation.
 * @param control_horizon_ Number of control steps to be considered.
 * @param block_length_   Block length representing the segmentation of the control sequence.
 *
 * Usage:
 * When used with Ceres, an instance of GoalAlignCost is wrapped in a ceres::AutoDiffCostFunction
 * and added as a residual block in the optimization problem.
 */
namespace nav2_social_mpc_controller
{

class GoalAlignCost
{
  /**
   * @class GoalAlignCost
   * @brief Functor for computing the cost of aligning the robot's heading with a goal heading.
   *
   * This class defines a cost function that computes the cost based on the robot's heading alignment
   * with a goal heading. It uses automatic differentiation to compute the residuals for optimization.
   */
public:
  using GoalAlignCostFuction = ceres::DynamicAutoDiffCostFunction<GoalAlignCost>;
  GoalAlignCost(double weight, const Eigen::Matrix<double, 2, 1> goal_heading,
                const geometry_msgs::msg::Pose& robot_init, unsigned int current_position, double time_step,
                unsigned int control_horizon, unsigned int block_length);

  /**
   * @brief Create a Ceres cost function for goal alignment cost.
   *
   * This function creates a Ceres cost function for goal alignment cost using automatic differentiation.
   * It initializes the cost function with the current instance of GoalAlignCost.
   * @return A pointer to the created AutoDiffCostFunction instance.
   */
  inline static GoalAlignCostFuction* Create(double weight, const Eigen::Matrix<double, 2, 1> goal_heading,
                                             const geometry_msgs::msg::Pose& robot_init, unsigned int current_position,
                                             double time_step, unsigned int control_horizon, unsigned int block_length)
  {
    // The first number is the number of cost functions.
    // The following number are the length of the parameters passed in the
    // addResidualBlock function.
    return new GoalAlignCost::GoalAlignCostFuction(new GoalAlignCost(weight, goal_heading, robot_init, current_position,
                                                                     time_step, control_horizon, block_length));
  }

  /**
   * @brief operator() computes the cost based on the robot's heading alignment with a goal heading.
   *
   * This function computes the cost based on the robot's heading alignment with a goal heading.
   * It propagates the robot state using the provided parameters and computes the residuals
   * based on the difference between the robot's heading and the goal heading.
   *
   * @param parameters Pointer to the array of parameters (robot state).
   * @param residuals Pointer to the output residuals array.
   * @return true if the operation was successful, false otherwise.
   */
  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    // calling the function to propagate forward the state, adding the subsequent velocity commands
    // to the robot state
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
        robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);

    auto turning_angle = T(0.0);
    // compute the turning angle, difference between the goal heading and the robot heading

    turning_angle = ceres::atan2(ceres::sin(goal_heading_[1] - new_position_orientation),
                                 ceres::cos(goal_heading_[1] - new_position_orientation));
    residuals[0] = (T)weight_ * turning_angle * turning_angle;

    return true;
  }

private:
  // parameters list
  double weight_;
  Eigen::Matrix<double, 2, 1> goal_heading_;
  geometry_msgs::msg::Pose robot_init_;
  AgentsStates agents_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
};

}  // namespace nav2_social_mpc_controller

#endif