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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__DISTANCE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__DISTANCE_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "nav2_social_mpc_controller/update_state.hpp"
/**
 * @file distance_cost_function.hpp
 * @brief Defines the DistanceCostFunction for social MPC control.
 *
 * This file declares the DistanceCostFunction struct, which computes a residual representing
 * the deviation of the robot's predicted state from a target point over a given horizon.
 *
 * The cost is evaluated by:
 * - Calculating the updated state of the robot starting from an initial pose.
 * - Propagating the state over a number of time steps using provided control inputs.
 * - Comparing the final predicted position with a reference point.
 * - Computing the residual as the weighted fourth power of the Euclidean distance between 
 *   the predicted position and the target position.
 *
 * The function is templated to support automatic differentiation with the Ceres Solver.
 *
 * @tparam T Scalar type used in the evaluation, typically double, to enable the use of Ceres autodiff.
 *
 * @param parameters A pointer-to-pointer representing the array of control input parameters 
 *                   (e.g., velocity and angular rate) applied at each step of the state update.
 * @param residuals  A pointer to the computed cost residual.
 *
 * @note The weight factor, control horizon, block length, and time step parameters are used
 *       to adjust how the cost is computed and influence the convergence of the optimization.
 *
 * Usage of this cost function is typically within an optimization problem solved using Ceres,
 * where it helps maintain the planned path close to desired waypoints or the goal.
 *
 * @author
 * @date
 */
namespace nav2_social_mpc_controller
{

class DistanceCost
{
public:
  using DistanceCostFunction = ceres::DynamicAutoDiffCostFunction<DistanceCost>;
  /**
    * @brief Constructor for the DistanceCost.
    *
    * Initializes the functor with a weight, target point, initial robot pose,
    * current position in the planning sequence, time step, control horizon,
    * and block length.
    *
    * @param weight The weight for the cost function.
    * @param point The target point towards which the robot should align its heading.
    * @param robot_init The initial pose of the robot.
    * @param current_position The current position in the planning sequence.
    * @param time_step The time step for the MPC.
    * @param control_horizon The total number of time steps in the MPC.
    * @param block_length The length of the parameter block for the MPC.
    */
  DistanceCost(
    double weight, const Eigen::Matrix<double, 2, 1> point,
    const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
    unsigned int control_horizon, unsigned int block_length);

  /**
    * @brief Creates a Ceres cost function for the DistanceCost.
    *
    * This function is a factory method that constructs an instance of the
    * DistanceCost using the provided parameters.
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
  inline static DistanceCostFunction * Create(
    double weight, const Eigen::Matrix<double, 2, 1> point,
    const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
    unsigned int control_horizon, unsigned int block_length)
  {
    return new DistanceCostFunction(new DistanceCost(
      weight, point, robot_init, current_position, time_step, control_horizon, block_length));
  }

  /**
    * @brief Operator to compute the distance cost based on the robot's state and target point.
    *
    * This function computes the residual representing the squared distance between the
    * predicted position of the robot after applying control inputs and a target point.
    * The updated state is computed using the current robot pose and control parameters
    * over a defined time step and control horizon.
    *
    * @param parameters Double pointer to robot state parameters.
    * @param residuals Pointer to the computed residual used in the optimization problem.
    * @return True if successful, false otherwise.
    */
  template <typename T>
  bool operator()(T const * const * parameters, T * residuals) const
  {
    // calling the function to propagate forward the state, adding the subsequent velocity commands
    // to the robot state

    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
      robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);

    // Now compute the residual based on the final state and target point.
    Eigen::Matrix<T, 2, 1> p((T)new_position_x, (T)new_position_y);
    Eigen::Matrix<T, 2, 1> p_target((T)point_[0], (T)point_[1]);
    residuals[0] = T(weight_) * (p - p_target).squaredNorm() * (p - p_target).squaredNorm();

    return true;
  }

private:
  unsigned int block_length_;
  unsigned int control_horizon_;
  double time_step_;
  unsigned int current_position_;
  double weight_;
  const Eigen::Matrix<double, 2, 1> point_;
  geometry_msgs::msg::Pose robot_init_;
};

}  // namespace nav2_social_mpc_controller

#endif