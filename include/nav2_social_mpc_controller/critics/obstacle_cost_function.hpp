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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_COST_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_COST_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "geometry_msgs/msg/pose.hpp"
#include "glog/logging.h"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_social_mpc_controller/update_state.hpp"

/**
 * @brief Functor for computing the obstacle cost residual for a given robot state.
 *
 * This cost function is used within a model predictive control (MPC) framework
 * to evaluate the proximity of a robot to obstacles on a 2D costmap. It uses a
 * bi-cubic interpolator to query costmap values at both the robot's current position
 * and at an offset position ahead of the robot (representing the front) based on its orientation.
 *
 * The residual is computed as a weighted sum of the interpolated costs.
 * It is designed for use with the Ceres Solver, enabling automatic differentiation
 * and optimization in trajectory planning applications.
 *
 * @note This struct is intended for use with costmap-based obstacle avoidance
 * and can handle dynamic updating of robot states over a control horizon.
 *
 * @param weight The weighting factor applied to the obstacle cost values.
 * @param costmap A pointer to the 2D costmap used for evaluating obstacles.
 * @param costmap_interpolator A shared pointer to a bi-cubic interpolator for
 *        performing costmap value queries with subpixel accuracy.
 * @param robot_init The initial pose of the robot.
 * @param current_position The index of the current position in the trajectory.
 * @param time_step The time interval used for state updates.
 * @param control_horizon The length of the control horizon.
 * @param block_length The block length in the optimization problem.
 *
 * @tparam T The type used for automatic differentiation (typically double).
 *
 * @return true if the cost is evaluated successfully, false otherwise.
 *
 * Usage Example:
 * @code
 *   nav2_social_mpc_controller::ObstacleCost cost_function(
 *       weight, costmap, costmap_interpolator, robot_init, current_position,
 *       time_step, control_horizon, block_length);
 *
 *   // Assume parameters is an array of pointers to the state variables.
 *   double residual;
 *   cost_function(parameters, &residual);
 * @endcode
 */
namespace nav2_social_mpc_controller
{

class ObstacleCost
{
public:
  using ObstacleCostFunction = ceres::DynamicAutoDiffCostFunction<ObstacleCost>;

  /**
  * @brief Constructor for the ObstacleCost.
  *
  * Initializes the functor with a weight, costmap, costmap interpolator,
  * initial robot pose, current position in the planning sequence, time step,
  * control horizon, and block length.
  *
  * @param weight The weight for the cost function.
  * @param costmap A pointer to the 2D costmap used for evaluating obstacles.
  * @param costmap_interpolator A shared pointer to a bi-cubic interpolator for
  *        performing costmap value queries with subpixel accuracy.
  * @param robot_init The initial pose of the robot.
  * @param current_position The current position in the planning sequence.
  * @param time_step The time step for the MPC.
  * @param control_horizon The total number of time steps in the MPC.
  * @param block_length The length of the parameter block for the MPC.
  */
  ObstacleCost(
    double weight, const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
    const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
    unsigned int control_horizon, unsigned int block_length);

  /**
  * @brief Creates a Ceres cost function for the ObstacleCost.
  *
  * This function is a factory method that constructs an instance of the
  * ObstacleCost using the provided parameters.
  *
  * @param weight The weight for the cost function.
  * @param costmap A pointer to the 2D costmap used for evaluating obstacles.
  * @param costmap_interpolator A shared pointer to a bi-cubic interpolator for
  *        performing costmap value queries with subpixel accuracy.
  * @param robot_init The initial pose of the robot.
  * @param current_position The current position in the planning sequence.
  * @param time_step The time step for the MPC.
  * @param control_horizon The total number of time steps in the MPC.
  * @param block_length The length of the parameter block for the MPC.
  * @return A pointer to the created AutoDiffCostFunction instance.
  */
  inline static ObstacleCostFunction * Create(
    double weight, const nav2_costmap_2d::Costmap2D * costmap,
    const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
    const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
    unsigned int control_horizon, unsigned int block_length)
  {
    return new ObstacleCostFunction(new ObstacleCost(
      weight, costmap, costmap_interpolator, robot_init, current_position, time_step,
      control_horizon, block_length));
  }

  /**
  * @brief operator() to compute the cost based on the robot's state and costmap.
  *
  * This function computes the residuals based on the robot's position and orientation,
  * using bi-cubic interpolation to evaluate the costmap at the robot's position and a point
  * in front of the robot. The residuals are scaled by a weight factor.
  *
  * @param parameters Pointer to the array of parameters (robot state).
  * @param residuals Pointer to the computed residuals.
  * @return True if the cost is evaluated successfully, false otherwise.
  */

  template <typename T>
  bool operator()(T const * const * parameters, T * residuals) const
  {
    // calling the function to propagate forward the state, adding the subsequent velocity commands
    // to the robot state
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
      robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);

    // starting from the robot position, we compute the interpolated cost using bi-cubic interpolation
    // for the robot position and the front position
    // the front position is computed by adding a small offset in the direction of the robot orientation
    // the offset is a constant value, considering the size of the robot
    // the costmap is a 2D grid, so we need to convert the position to the grid coordinates.

    T value_front;
    const T front_offset = T(0.25);  // considering size of jackal

    T front_x = new_position_x + front_offset * ceres::cos(new_position_orientation);
    T front_y = new_position_y + front_offset * ceres::sin(new_position_orientation);

    Eigen::Matrix<T, 2, 1> front(front_x, front_y);
    Eigen::Matrix<T, 2, 1> interp_front =
      (front - costmap_origin_.template cast<T>()) / (T)costmap_resolution_;

    costmap_interpolator_->Evaluate(interp_front[1], interp_front[0], &value_front);

    // the residual is the sum of the two costs, multiplied by the weight
    residuals[0] = (T)weight_ * value_front;  //+ (T)weight_ *value;

    return true;
  }

  unsigned int block_length_;
  unsigned int control_horizon_;
  double time_step_;
  unsigned int current_position_;
  double weight_;
  geometry_msgs::msg::Pose robot_init_;
  Eigen::Vector2d costmap_origin_;
  double costmap_resolution_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> costmap_interpolator_;
};

}  // namespace nav2_social_mpc_controller

#endif