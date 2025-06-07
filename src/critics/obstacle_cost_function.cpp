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

#include "nav2_social_mpc_controller/critics/obstacle_cost_function.hpp"
namespace nav2_social_mpc_controller
{

ObstacleCost::ObstacleCost(
  double weight, const nav2_costmap_2d::Costmap2D * costmap,
  const std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<u_char>>> & costmap_interpolator,
  const geometry_msgs::msg::Pose & robot_init, unsigned int current_position, double time_step,
  unsigned int control_horizon, unsigned int block_length)
: weight_(weight),
  costmap_origin_(costmap->getOriginX(), costmap->getOriginY()),
  costmap_resolution_(costmap->getResolution()),
  costmap_interpolator_(costmap_interpolator)
{
  control_horizon_ = control_horizon;
  robot_init_ = robot_init;
  current_position_ = current_position;
  time_step_ = time_step;
  block_length_ = block_length;
}

}  // namespace nav2_social_mpc_controller
