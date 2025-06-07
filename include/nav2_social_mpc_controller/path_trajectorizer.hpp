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

#ifndef NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
#define NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
//#include "nav2_core/smoother.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_social_mpc_controller
{

/**
 * @class nav2_path_trajectorizer::PathTrajectorizer
 * @brief Regulated pure pursuit controller plugin
 */
class PathTrajectorizer
{
public:
  /**
   * @brief Constructor for nav2_path_trajectorizer::PathTrajectorizer
   */
  PathTrajectorizer();

  /**
   * @brief Destrructor for nav2_path_trajectorizer::PathTrajectorizer
   */
  ~PathTrajectorizer();

  /**
   * @brief Configure smoother parameters and member variables
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   */
  void configure(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf);
  // std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
  // std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> footprint_sub);

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup();

  /**
   * @brief Activate controller state machine
   */
  void activate();

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate();

  /**
   * @brief Method to smooth given path
   *
   * @param path In-out path to be optimized
   * @param path_robot_pose, pose of the robot in the same frame of the path
   * @return trajectorized path
   */
  bool trajectorize(nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& path_robot_pose,
                    std::vector<geometry_msgs::msg::TwistStamped>& cmds);

  float inline getTimeStep()
  {
    return time_step_;
  }

protected:
  /**
   * @brief  Compute x position based on velocity
   * @param  xi The current x position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new x position
   */
  inline double computeNewXPosition(double xi, double vx, double vy, double theta, double dt)
  {
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;  //
  }

  /**
   * @brief  Compute y position based on velocity
   * @param  yi The current y position
   * @param  vx The current x velocity
   * @param  vy The current y velocity
   * @param  theta The current orientation
   * @param  dt The timestep to take
   * @return The new y position
   */
  inline double computeNewYPosition(double yi, double vx, double vy, double theta, double dt)
  {
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
  }

  /**
   * @brief  Compute orientation based on velocity
   * @param  thetai The current orientation
   * @param  vth The current theta velocity
   * @param  dt The timestep to take
   * @return The new orientation
   */
  inline double computeNewThetaPosition(double thetai, double vth, double dt)
  {
    return thetai + vth * dt;
  }

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  rclcpp::Logger logger_{ rclcpp::get_logger("PathTrajectorizer") };
  rclcpp::Clock::SharedPtr clock_;

  // std::unique_ptr<nav2_constrained_smoother::Smoother> smoother_;
  // SmootherParams smoother_params_;
  // OptimizerParams optimizer_params_;

  double desired_linear_vel_;
  double lookahead_dist_;
  double max_angular_vel_;
  bool omnidirectional_;
  double time_step_;
  double max_steps_;
  rclcpp::Duration transform_tolerance_{ 0, 0 };
  std::string base_frame_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> received_path_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> computed_path_pub_;
  // double rotate_to_heading_angular_vel_;
  // double max_lookahead_dist_;
  // double min_lookahead_dist_;
  // double lookahead_time_;
};

}  // namespace nav2_social_mpc_controller

#endif  // NAV2_CONSTRAINED_SMOOTHER__CONSTRAINED_SMOOTHER_HPP_
