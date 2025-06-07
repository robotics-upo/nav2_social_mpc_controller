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

#include "nav2_social_mpc_controller/critics/agent_angle_cost_function.hpp"

namespace nav2_social_mpc_controller
{

AgentAngleCost::AgentAngleCost(double weight, const AgentsStates& agents_init,
                               const geometry_msgs::msg::Pose& robot_init, unsigned int current_position,
                               double time_step, unsigned int control_horizon, unsigned int block_length)
  : weight_(weight)
  , agents_init_(agents_init)
  , robot_init_(robot_init)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
{
  safe_distance_squared_ = 4.0;
}

}  // namespace nav2_social_mpc_controller