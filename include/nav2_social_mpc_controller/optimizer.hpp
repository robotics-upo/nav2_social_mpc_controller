// Copyright (c) 2022 SRL -Service Robotics Lab, Pablo de Olavide University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// ributed under the License is ributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__OPTIMIZER_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__OPTIMIZER_HPP_

#include <math.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "ceres/cost_function.h"
#include "ceres/cubic_interpolation.h"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

// cost functions
#include "nav2_social_mpc_controller/critics/agent_angle_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/angle_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/curvature_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/distance_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/goal_align_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/obstacle_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/social_work_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/velocity_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/velocity_feasibility_cost_function.hpp"
#include "nav2_social_mpc_controller/critics/proxemics_cost_function.hpp"

#include "nav2_social_mpc_controller/sfm.hpp"
#include "nav2_social_mpc_controller/trajectory_memory.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "people_msgs/msg/people.hpp"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"

namespace nav2_social_mpc_controller
{

struct OptimizerParams
{
  OptimizerParams()
  {
  }

  /**
   * @brief Get params from ROS parameter
   * @param node Ptr to node
   * @param name Name of plugin
   */
  void get(rclcpp_lifecycle::LifecycleNode* node, const std::string& name);
  const std::map<std::string, ceres::LinearSolverType> solver_types = {
    { "DENSE_SCHUR", ceres::DENSE_SCHUR },
    { "SPARSE_SCHUR", ceres::SPARSE_SCHUR },
    { "DENSE_NORMAL_CHOLESKY", ceres::DENSE_NORMAL_CHOLESKY },
    { "DENSE_QR", ceres::DENSE_QR },
    { "SPARSE_NORMAL_CHOLESKY", ceres::SPARSE_NORMAL_CHOLESKY }
  };

  std::string linear_solver_type;

  double param_tol;     // Ceres default: 1e-8
  double fn_tol;        // Ceres default: 1e-6
  double gradient_tol;  // Ceres default: 1e-10
  double socialwork_w_;
  double distance_w_;
  double velocity_w_;
  double angle_w_;
  double agent_angle_w_;
  double velocity_feasibility_w_;
  double goal_align_w_;
  double obstacle_w_;
  double proxemics_w_;
  float current_path_w;
  float current_cmds_w;
  float max_time;
  int discretization_;
  int control_horizon_;
  int parameter_block_length_;
  bool debug;
  int max_iterations;
};

/**
 * @brief Optimizer class for the social MPC controller
 */
class Optimizer
{
public:
  // x, y
  struct position
  {
    double params[2];
  };

  // x, y, lv, av
  struct posandvel
  {
    double params[4];
  };

  // lv, av
  struct vel
  {
    double params[2];
  };

  // t, yaw
  struct heading
  {
    double params[2];
  };
  struct linear_velocity
  {
    double params[1];
  };
  struct angular_velocity
  {
    double params[1];
  };
  Optimizer();

  /**
   * @brief Destrructor for
   * nav2_social_mpc_controller::SocialMPCController
   */
  ~Optimizer();

  /**
   * @brief Initialization of the optimizer
   * @param params OptimizerParam struct
   */
  void initialize(const OptimizerParams params);

  /**
   * @brief Optimize the path using the social MPC controller
   * @param path The path to optimize
   * @param people_proj Projected people positions
   * @param costmap Costmap for obstacle avoidance
   * @param obstacles Obstacle distances
   * @param cmds Commands to execute
   * @param people People detected in the environment
   * @param speed Current robot speed
   * @param time_step Time step for discretization
   * @return true if optimization succeeded
   * @return false if optimization failed
   */
  bool optimize(nav_msgs::msg::Path& path, AgentsTrajectories& people_proj, const nav2_costmap_2d::Costmap2D* costmap,
                const obstacle_distance_msgs::msg::ObstacleDistance& obstacles,
                std::vector<geometry_msgs::msg::TwistStamped>& cmds, const people_msgs::msg::People& people,
                const geometry_msgs::msg::Twist& speed, const float time_step);

private:
  /**
   * @brief Convert people messages to agent status
   * @param people People messages
   * @return Vector of agent statuses
   */
  AgentsStates people_to_status(const people_msgs::msg::People& people);

  /**
   * @brief Format path and commands for optimization
   * @param path Current path
   * @param previous_path Previous path
   * @param cmds Current commands
   * @param previous_cmds Previous commands
   * @param speed Current robot speed
   * @param current_path_w Weight for current path
   * @param current_cmds_w Weight for current commands
   * @param maxtime Maximum time horizon
   * @param timestep Time step
   * @return Vector of agent statuses
   */
  AgentTrajectory format_to_optimize(nav_msgs::msg::Path& path, const nav_msgs::msg::Path& previous_path,
                                     const std::vector<geometry_msgs::msg::TwistStamped>& cmds,
                                     const std::vector<geometry_msgs::msg::TwistStamped>& previous_cmds,
                                     const geometry_msgs::msg::Twist& speed, const float current_path_w,
                                     const float current_cmds_w, const float maxtime, const float timestep);

  /**
   * @brief Project people positions over time
   * @param init_people Initial people positions
   * @param robot_path Robot path
   * @param od Obstacle distances
   * @param maxtime Maximum time horizon
   * @param timestep Time step
   * @return Vector of vector of agent statuses
   */
  AgentsTrajectories project_people(const AgentsStates& init_people, const AgentTrajectory& robot_path,
                                    const obstacle_distance_msgs::msg::ObstacleDistance& od, const float& maxtime,
                                    const float& timestep);

  /**
   * @brief Compute obstacle position relative to agent
   * @param apos Agent position
   * @param od Obstacle distances
   * @return Vector to obstacle
   */
  Eigen::Vector2d computeObstacle(const Eigen::Vector2d& apos, const obstacle_distance_msgs::msg::ObstacleDistance& od);

  bool debug_;
  unsigned int control_horizon_;
  unsigned int parameter_block_length_;
  float max_time;
  double obstacle_w_;
  double velocity_feasibility_w_;
  double agent_angle_w_;
  double angle_w_;
  double distance_w_;
  double socialwork_w_;
  double goal_align_w_;
  double velocity_w_;
  double curvature_w_;
  double proxemics_w_;
  double curvature_angle_min_;
  float current_path_w;
  float current_cmds_w;
  ceres::Solver::Options options_;
  std::shared_ptr<ceres::Grid2D<u_char>> costmap_grid_;
  std::shared_ptr<ceres::Grid2D<float>> obs_grid_;
  std::string frame_;
  rclcpp::Time path_time_;
};

}  // namespace nav2_social_mpc_controller

#endif  // NAV2_SOCIAL_MPC_CONTROLLER__OPTIMIZER_HPP_