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

#include "nav2_social_mpc_controller/social_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "angles/angles.h"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using std::abs;
using std::hypot;
using std::max;
using std::min;
using namespace nav2_costmap_2d;  // NOLINT

double clamp(double value, double min, double max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

namespace nav2_social_mpc_controller
{

void SocialMPCController::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                                    std::shared_ptr<tf2_ros::Buffer> tf,
                                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  double transform_tolerance;
  declare_parameter_if_not_declared(node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(node, plugin_name_ + ".fov_angle", rclcpp::ParameterValue(M_PI / 4));
  declare_parameter_if_not_declared(node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = tf2::durationFromSec(transform_tolerance);
  node->get_parameter(plugin_name_ + ".fov_angle", fov_angle_);
  // Create the trajectorizer
  trajectorizer_ = std::make_unique<PathTrajectorizer>();
  trajectorizer_->configure(node, name, tf_);
  optimizer_ = std::make_unique<Optimizer>();
  optimizer_params_.get(node.get(), name);
  optimizer_->initialize(optimizer_params_);

  // people interface
  people_interface_ = std::make_unique<PeopleInterface>(parent);

  // path handler
  path_handler_ = std::make_unique<mpc::PathHandler>(transform_tolerance_, tf_, costmap_ros_);

  // obstacle distance transform
  obsdist_interface_ =
      std::make_unique<ObstacleDistInterface>(node, costmap_ros_->getGlobalFrameID(), tf_, transform_tolerance_);

  local_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);

  people_traj_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("people_projected_trajectory", 1);
}

void SocialMPCController::cleanup()
{
  RCLCPP_INFO(logger_,
              "Cleaning up controller: %s of type"
              "nav2_social_mpc_controller::SocialMPCController",
              plugin_name_.c_str());
  local_path_pub_.reset();
  people_traj_pub_.reset();
}

void SocialMPCController::activate()
{
  RCLCPP_INFO(logger_,
              "Activating controller: %s of type "
              "nav2_social_mpc_controller::SocialMPCController",
              plugin_name_.c_str());
  trajectorizer_->activate();
  local_path_pub_->on_activate();
  people_traj_pub_->on_activate();
}

void SocialMPCController::deactivate()
{
  RCLCPP_INFO(logger_,
              "Deactivating controller: %s of type "
              "nav2_social_mpc_controller::SocialMPCController",
              plugin_name_.c_str());
  trajectorizer_->deactivate();
  local_path_pub_->on_deactivate();
  people_traj_pub_->on_deactivate();
}

void SocialMPCController::publish_people_traj(const AgentsTrajectories& people, const std_msgs::msg::Header& header)
{
  // Create one marker for each person
  size_t npeople = people[0].size();
  visualization_msgs::msg::MarkerArray ma;
  for (size_t idx = 0; idx < npeople; idx++)
  {
    if (people[0][idx][3] != -1.0)
    {
      visualization_msgs::msg::Marker m;
      m.header = header;
      m.type = m.LINE_STRIP;
      m.id = idx;
      m.action = m.ADD;
      m.scale.x = 0.05;
      m.color.a = 1.0;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      ma.markers.push_back(m);
    }
  }

  for (unsigned int stepi = 0; stepi < people.size(); stepi++)
  {
    int mi = 0;
    for (unsigned int personi = 0; personi < people[stepi].size(); personi++)
    {
      if (people[stepi][personi][3] != -1.0)
      {
        geometry_msgs::msg::Point point;
        point.x = people[stepi][personi][0];
        point.y = people[stepi][personi][1];
        point.z = 0.1;
        ma.markers[mi].points.push_back(point);
        mi++;
      }
    }
  }
  people_traj_pub_->publish(ma);
}

geometry_msgs::msg::TwistStamped SocialMPCController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& robot_pose, const geometry_msgs::msg::Twist& speed,
    nav2_core::GoalChecker* goal_checker)
{
  // Use goal_checker to avoid unused parameter warning
  if (goal_checker == nullptr)
  {
    RCLCPP_WARN(logger_, "Goal checker is null");
  }
  nav_msgs::msg::Path transformed_plan =
      path_handler_->transformGlobalPlan(robot_pose, 4.0);  // TODO: make this a parameter
  auto goal = path_handler_->getTransformedGoal(2.5, transformed_plan, robot_pose);

  // Trajectorize the path
  nav_msgs::msg::Path traj_path = transformed_plan;
  std::vector<geometry_msgs::msg::TwistStamped> cmds;
  // trajectorizer_->trajectorize(traj_path, robot_pose, cmds);

  if (!trajectorizer_->trajectorize(traj_path, robot_pose, cmds))
  {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header = robot_pose.header;
    cmd_vel.twist.linear.x = 0.1;  // Use the desired speed as a fallback
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;  // No angular velocity
    RCLCPP_WARN(logger_, "Approaching goal without a valid trajectory, using fallback cmd_vel");
    return cmd_vel;
  }
  std::vector<geometry_msgs::msg::TwistStamped> init_cmds = cmds;
  // float goal_distance = euclidean_distance(goal.point, robot_pose.pose.position);

  // Be careful, path and people must be in the same frame
  people_msgs::msg::People people_unf = people_interface_->getPeople();
  people_msgs::msg::People people;

  // only use people in the FOV of the robot, in this case (-90°,90° supposed )
  for (auto p : people_unf.people)
  {
    uint mx, my;
    if (!costmap_->worldToMap(p.position.x, p.position.y, mx, my))
    {
      RCLCPP_DEBUG(logger_, "Person %s is not in the costmap", p.name.c_str());
      continue;
    }
    float angle_to_person = atan2(p.position.y - robot_pose.pose.position.y, p.position.x - robot_pose.pose.position.x);
    float robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    float relative_angle = angles::shortest_angular_distance(robot_yaw, angle_to_person);
    if (fabs(relative_angle) < fov_angle_)
    {
      people.people.push_back(p);
    }
    // Filter people based on the FOV of the robot
  }
  people.header.frame_id = people_unf.header.frame_id;

  if (people.header.frame_id != transformed_plan.header.frame_id)
  {
    // transform people to the global frame
    for (auto p : people.people)
    {
      geometry_msgs::msg::PointStamped out_point;
      geometry_msgs::msg::PointStamped in_point;
      in_point.point = p.position;
      in_point.header = people.header;
      if (!transformPoint(transformed_plan.header.frame_id, in_point, out_point))
      {
        throw nav2_core::PlannerException("Unable to transform people point into plan's frame");
      }
      p.position = out_point.point;
    }
  }

  // Get the distance transform
  obstacle_distance_msgs::msg::ObstacleDistance transformed_od = obsdist_interface_->getDistanceTransform();

  float ts = trajectorizer_->getTimeStep();
  AgentsTrajectories projected_people;

  bool optimized = optimizer_->optimize(traj_path, projected_people, costmap_, transformed_od, cmds, people, speed, ts);
  if (!optimized)
  {
    RCLCPP_WARN(logger_, "Optimization failed, using initial commands");
    cmds = init_cmds;
  }
  publish_people_traj(projected_people, transformed_plan.header);
  local_path_pub_->publish(traj_path);

  // populate and return twist message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = cmds[0].header;
  cmd_vel.twist.linear.x = cmds[0].twist.linear.x;
  cmd_vel.twist.linear.y = 0;
  cmd_vel.twist.angular.z = cmds[0].twist.angular.z;
  RCLCPP_DEBUG(logger_, "cmd_vel: %f, %f", cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
  return cmd_vel;
}

void SocialMPCController::setPlan(const nav_msgs::msg::Path& path)
{
  path_handler_->setPlan(path);
}

void SocialMPCController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  double speed_limit_ = speed_limit;
  bool percentage_ = percentage;
  speed_limit_ = 0;
  percentage_ = false;
  double throwaway_vel = 1;
  // RCLCPP_DEBUG(logger_, "Setting speed limit to %f, percentage %", speed_limit_);
  if (percentage_)
  {
    throwaway_vel *= (speed_limit_ / 100.0);
    RCLCPP_DEBUG(logger_, "Speed limit set as percentage: %f%%, resulting speed: %f", speed_limit_,
                 desired_linear_vel_);
  }
  else
  {
    throwaway_vel = speed_limit_;
    RCLCPP_DEBUG(logger_, "Speed limit set as absolute value: %f", desired_linear_vel_);
  }
}

bool SocialMPCController::transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                                        geometry_msgs::msg::PoseStamped& out_pose) const
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf_->transform(in_pose, out_pose, frame, transform_tolerance_);
    out_pose.header.frame_id = frame;
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

bool SocialMPCController::transformPoint(const std::string frame, const geometry_msgs::msg::PointStamped& in_point,
                                         geometry_msgs::msg::PointStamped& out_point) const
{
  try
  {
    tf_->transform(in_point, out_point, frame, transform_tolerance_);
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Exception in transformPoint: %s", ex.what());
  }
  return false;
}

}  // namespace nav2_social_mpc_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_social_mpc_controller::SocialMPCController, nav2_core::Controller)
