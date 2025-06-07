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

#include "nav2_social_mpc_controller/critics/proxemics_cost_function.hpp"

namespace nav2_social_mpc_controller
{

ProxemicsCost::ProxemicsCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                             const double counter, unsigned int current_position, double time_step,
                             unsigned int control_horizon, unsigned int block_length)
  : weight_(weight)
  , robot_init_(robot_init)
  , counter_(counter)
  , current_position_(current_position)
  , time_step_(time_step)
  , control_horizon_(control_horizon)
  , block_length_(block_length)
{
  for (unsigned int j = 0; j < agents_init.size(); j++)
  {
    original_agents_.col(j) << agents_init[j][0], agents_init[j][1], agents_init[j][2], agents_init[j][3],
        agents_init[j][4], agents_init[j][5];
  }

  alpha_ = 3.0;  // Scaling factor for the proxemics cost
  d0_ = 0.5;     // Minimum distance for proxemics cost
}

}  // namespace nav2_social_mpc_controller
