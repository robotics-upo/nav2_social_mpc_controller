// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University and PIC4SeR - Politecnico di Torino
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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__AGENT_ANGLE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__AGENT_ANGLE_COST_FUNCTION_HPP_

#include <nav2_social_mpc_controller/update_state.hpp>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"

/**
 * @file agent_angle_cost_function.hpp
 * @brief Defines the cost function for evaluating robot steering adjustments based on nearby agent orientations.
 *
 * This file implements the AgentAngleCost struct which serves as a functor for calculating a cost
 * that incentives certain angular deviations in a model predictive control (MPC) framework for social navigation.
 *
 * The cost is evaluated only when a nearby agent (satisfying certain activity and proximity thresholds)
 * is present. The function wraps angular differences to the range [-pi, pi] and adjusts its computation
 * based on the relative heading of the agent with respect to the robot's initial pose.
 *
 * The cost function leverages Ceres Solver for automatic differentiation and angle operations, and is
 * designed to be integrated within an optimization problem formulated to improve robot trajectory planning
 * in social environments.
 *
 * @note This implementation assumes that the state update computed by computeUpdatedStateRedux and other
 * necessary external functions or variables (e.g., tf2::getYaw) are defined elsewhere.
 *
 * @license Apache License, Version 2.0
 */

namespace nav2_social_mpc_controller
{

class AgentAngleCost
{
  /**
   * @class AgentAngleCost
   * @brief Functor for computing angular cost based on relative positions and orientations of agents.
   *
   * The AgentAngleCost computes the residual cost that penalizes the deviation of the robot's new orientation
   * from an expected angular value determined by the robot's initial orientation and the heading of a close-by agent.
   *
   * The function finds the nearest agent that satisfies a minimum linear velocity threshold, and if the distance is
   * within a predefined safe threshold, the cost is computed. The angular correction is applied differently based on
   * the relative orientation of the agent with respect to the robot’s initial yaw.
   *
   * @param weight A scaling factor for the cost.
   * @param agents_init A vector of agent statuses representing their initial state. Each agent status is a 6x1 Eigen
   * vector.
   * @param agents_zero A vector of baseline (zero or near-zero) agent statuses used for comparison.
   * @param robot_init The initial pose of the robot, which includes its position and orientation.
   * @param current_position The current index or time step in the planning horizon.
   * @param time_step The time interval between consecutive state updates.
   * @param control_horizon The total number of time steps considered in the MPC.
   * @param block_length The segmentation length used in state update computations.
   *
   * Member Variables:
   * - block_length_: Number of time steps in a block for updating the state.
   * - control_horizon_: Total number of steps in the MPC planning horizon.
   * - time_step_: Duration between state updates.
   * - current_position_: Index marking the current position in the planning sequence.
   * - safe_distance_: The threshold distance below which agent interactions trigger cost calculations.
   * - weight_: The overall scaling weight applied to the computed cost.
   * - robot_init_: The initial pose of the robot.
   * - agents_init_: List of initial statuses (poses, velocities, etc.) of the agents.
   */
public:
  using AgentAngleCostFunction = ceres::DynamicAutoDiffCostFunction<AgentAngleCost>;
  AgentAngleCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                 unsigned int current_position, double time_step, unsigned int control_horizon,
                 unsigned int block_length);

  /**
    * @brief Creates a Ceres cost function for the AgentAngleCost.
    *
    * This function is a factory method that constructs an instance of the
    * AgentAngleCostFunction using the provided parameters.

    * @param weight The weight for the cost function.
    * @param agents_init A vector of initial agent statuses.
    * @param robot_init The initial pose of the robot.
    * @param current_position The current position in the planning sequence.
    * @param time_step The time step for the MPC.
    * @param control_horizon The total number of time steps in the MPC.
    * @param block_length The length of the parameter block for the MPC.
    * @return A pointer to the created AgentAngleCostFunction instance.
    */

  inline static AgentAngleCostFunction* Create(double weight, const AgentsStates& agents_init,
                                               const geometry_msgs::msg::Pose& robot_init,
                                               unsigned int current_position, double time_step,
                                               unsigned int control_horizon, unsigned int block_length)
  {
    return new AgentAngleCostFunction(new AgentAngleCost(weight, agents_init, robot_init, current_position, time_step,
                                                         control_horizon, block_length));
  }

  /**
   * @brief Operator to compute the cost based on the robot's state and agent positions.
   * @param T The type used for automatic differentiation (typically a double or Jet type from Ceres).
   * @param parameters Double pointer to robot state parameters.
   * @param residuals Pointer to the computed residual used in the optimization problem.
   *
   * The operator() function computes the updated state based on the input parameter block and evaluates
   * the angular difference with respect to a pre-determined steering offset (either left or right) relative to
   * the robot's initial orientation. If the nearest agent does not meet certain criteria (e.g., too far away,
   * inactive), the cost is set to zero.
   */
  template <typename T>
  bool operator()(T const* const* parameters, T* residuals) const
  {
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
        robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);
    int closest_index = -1;
    double closest_distance_squared = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < agents_init_.size(); ++i)
    {
      double dx = agents_init_[i][0] - robot_init_.position.x;
      double dy = agents_init_[i][1] - robot_init_.position.y;
      double distance_squared = dx * dx + dy * dy;
      if (distance_squared < closest_distance_squared && agents_init_[i][4] > 0.05)
      {
        closest_distance_squared = distance_squared;
        closest_index = i;
      }
    }
    if (closest_index < 0 || closest_distance_squared > safe_distance_squared_)
    {
      residuals[0] = T(0.0);
      return true;
    }
    AgentStatus closest_agent;
    closest_agent = agents_init_[closest_index];
    const auto& agent = closest_agent;
    // Compute angles.
    T agent_angle_initial = ceres::atan2(T(agent[1] - robot_init_.position.y), T(agent[0] - robot_init_.position.x));
    T robot_yaw = T(tf2::getYaw(robot_init_.orientation));
    // Difference between agent's heading and the robot's initial orientation.
    T agent_heading_diff = ceres::atan2(ceres::sin(T(agent[2]) - robot_yaw), ceres::cos(T(agent[2]) - robot_yaw));
    // Helper to wrap angles into [-pi, pi].
    auto wrapAngle = [](const T& angle) -> T { return ceres::atan2(ceres::sin(angle), ceres::cos(angle)); };

    // --- Define angular thresholds and offsets ---
    // Thresholds.
    const T kThreshold = T(M_PI / 6.0);
    const T kUpperThreshold = T(5 * M_PI / 6.0);
    const T steering_right = -T(M_PI / 6.0);
    const T steering_left = T(M_PI / 6.0);
    T angular_diff;

    if (agent_heading_diff <= -kUpperThreshold || agent_heading_diff >= kThreshold)
    {
      if (wrapAngle(agent_angle_initial - robot_yaw) < 0.0)
      {
        residuals[0] = T(0.0);
        return true;
      }
      else
      {
        angular_diff = wrapAngle(new_position_orientation - (robot_yaw + steering_right));
      }
    }

    else
    {
      if (wrapAngle(agent_angle_initial - robot_yaw) > 0.0)
      {
        residuals[0] = T(0.0);
        return true;
      }
      else
      {
        angular_diff = wrapAngle(new_position_orientation - (robot_yaw + steering_left));
      }
    }
    T cost = angular_diff * angular_diff;
    residuals[0] = weight_ * cost;
    return true;
  }

private:
  double weight_;
  AgentsStates agents_init_;
  geometry_msgs::msg::Pose robot_init_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  double safe_distance_squared_;
};

}  // namespace nav2_social_mpc_controller

#endif