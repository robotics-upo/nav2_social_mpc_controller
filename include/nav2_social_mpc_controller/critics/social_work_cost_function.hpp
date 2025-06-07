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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__SOCIAL_WORK_FUNCTION_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__SOCIAL_WORK_FUNCTION_HPP_

#include "Eigen/Core"
#include "ceres/ceres.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "glog/logging.h"
#include "nav2_social_mpc_controller/update_state.hpp"
#include "nav2_social_mpc_controller/tools/type_definitions.hpp"

namespace nav2_social_mpc_controller
{

template <typename T>
/**
 * @brief Wraps an angle to the range [-pi, pi].
 *
 * This function ensures that the angle is within the range of -pi to pi,
 * which is useful for angle normalization in trigonometric calculations.
 *
 * @param angle The angle in radians to be wrapped.
 * @return The wrapped angle in radians.
 */
inline T wrapToPi(T angle)
{
  while (angle > T(M_PI))
    angle -= T(2.0 * M_PI);
  while (angle <= T(-M_PI))
    angle += T(2.0 * M_PI);
  return angle;
}

class SocialWorkCost
{
  /**
   * @class SocialWorkCost
   * @brief Functor for computing the social work cost based on the robot's interaction with agents.
   *
   * This class implements a cost function that evaluates the social work done by the robot
   * in relation to other agents in the environment. It computes the social forces acting on
   * the robot and the agents, and calculates the residuals based on these forces.
   */
public:
  using SocialWorkCostFunction = ceres::DynamicAutoDiffCostFunction<SocialWorkCost>;

  SocialWorkCost(double weight, const AgentsStates& agents_init, const geometry_msgs::msg::Pose& robot_init,
                 const double counter, unsigned int current_position, double time_step, unsigned int control_horizon,
                 unsigned int block_length);

  /**
   * @brief Creates a Ceres cost function for the SocialWorkCost.
   *
   * This function is a factory method that constructs an instance of the
   * SocialWorkFunctionType, which is a Ceres cost function
   * for computing the social work cost based on the robot's interaction with agents.
   *
   * @param weight The weight of the cost function.
   * @param agents_init A vector of AgentStatus representing the initial states of the agents.
   * @param robot_init The initial pose of the robot.
   * @param counter A counter value used in the computation.
   * @param current_position The current position index in the control sequence.
   * @param time_step The time step for the state update.
   * @param control_horizon The total number of time steps in the MPC.
   * @param block_length The length of the parameter block for the MPC.
   * @return A pointer to the created SocialWorkFunctionType instance.
   */

  inline static SocialWorkCostFunction* Create(double weight, const AgentsStates& agents_init,
                                               const geometry_msgs::msg::Pose& robot_init, const double counter,
                                               unsigned int current_position, double time_step,
                                               unsigned int control_horizon, unsigned int block_length)
  {
    return new SocialWorkCostFunction(new SocialWorkCost(weight, agents_init, robot_init, counter, current_position,
                                                         time_step, control_horizon, block_length));
  }

  /**
   * @brief operator() computes the residual for the social work cost function.
   *
   * This function computes the updated state of the robot based on the input parameters and evaluates
   * the social forces acting on the robot and agents. It returns the residuals for optimization.
   *
   * @param parameters A pointer to an array of control parameters (e.g., velocities).
   * @param residual A pointer to store the computed residual value.
   * @return true if the computation was successful, false otherwise.
   */
  template <typename T>
  bool operator()(T const* const* parameters, T* residual) const
  {
    // Compute robot social work
    Eigen::Matrix<T, 6, 3> agents = original_agents_.template cast<T>();  // Convert original agents to type T
    Eigen::Matrix<T, 6, 1> robot;
    auto [new_position_x, new_position_y, new_position_orientation] = computeUpdatedStateRedux(
        robot_init_, parameters, time_step_, current_position_, control_horizon_, block_length_);  // Update robot state
    robot(0, 0) = (T)new_position_x;                                                               // x
    robot(1, 0) = (T)new_position_y;                                                               // y
    robot(2, 0) = (T)new_position_orientation;                                                     // yaw
    robot(3, 0) = (T)counter_;                                                                     // t
    if (current_position_ < control_horizon_)
    {
      robot(4, 0) = parameters[current_position_ / block_length_][0];  // lv
      robot(5, 0) = parameters[current_position_ / block_length_][1];  // av
    }
    else
    {
      robot(4, 0) = parameters[(control_horizon_ - 1) / block_length_][0];  // lv
      robot(5, 0) = parameters[(control_horizon_ - 1) / block_length_][1];  // av
    }

    Eigen::Matrix<T, 2, 1> robot_sf = computeSocialForce(robot, agents);  // Compute social force on robot
    T wr = (T)robot_sf.squaredNorm();  // Compute the squared norm of the social force on the robot

    // compute agents' social work provoked by the robot
    T wp = (T)0.0;
    Eigen::Matrix<T, 6, 3> robot_agent;
    robot_agent.col(0) << robot;  // Set the first column to the robot's current state
    // we invalidate the other two agent
    // by setting t to -1

    robot_agent.col(1) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;  // Set the second column to an invalid state
    robot_agent.col(2) << (T)0.0, (T)0.0, (T)0.0, (T)-1.0, (T)0.0, (T)0.0;  // Set the third column to an invalid state
    for (unsigned int i = 0; i < original_agents_.cols(); i++)              // Iterate through each agent
    {
      Eigen::Matrix<T, 6, 1> ag;                                // Create a matrix to hold the agent's state
      ag.col(0) << original_agents_.col(i).template cast<T>();  // Set the current state of the agent
      Eigen::Matrix<T, 2, 1> agent_sf = computeSocialForce(ag, robot_agent);  // Compute social force on agent
      wp += (T)agent_sf.squaredNorm();  // Accumulate the squared norm of the social force on the agent
    }
    T total_social_force_magnitude_sq = wr + wp + (T)1e-6;  // Avoid division by zero

    // sum the social works and multiply by the weight
    residual[0] = (T)weight_ * (total_social_force_magnitude_sq);

    return true;
  }

  /**
   * @brief This function computes the social force acting on the robot based on its position,
   * and the positions of other agents.
   * It calculates the interaction between the robot and other agents, taking into account their velocities and
   * the social force parameters.
   *
   * @tparam T
   * @param me the current state of the robot, including position, orientation, and velocity
   * @param agents the states of other agents in the environment, including their positions, orientations, and
   * velocities
   * @return Eigen::Matrix<T, 2, 1> the computed social force acting on the robot
   */
  template <typename T>
  Eigen::Matrix<T, 2, 1> computeSocialForce(const Eigen::Matrix<T, 6, 1>& me,
                                            const Eigen::Matrix<T, 6, 3>& agents) const
  {
    Eigen::Matrix<T, 2, 1> meSocialforce((T)0.0, (T)0.0);  // Initialize the social force vector
    Eigen::Matrix<T, 2, 1> mePos(me[0], me[1]);            // Extract the position of the robot
    Eigen::Matrix<T, 2, 1> meVel(me[4] * ceres::cos(me[2]),
                                 me[4] * ceres::sin(me[2]));  // Extract the velocity of the robot

    for (unsigned int i = 0; i < agents.cols(); i++)  // Iterate through each agent
    {
      if (agents(3, i) == (T)-1.0)  // Skip agents that are invalid (e.g., not present)
        continue;

      Eigen::Matrix<T, 2, 1> aPos(agents(0, i), agents(1, i));  // Extract the position of the agent
      Eigen::Matrix<T, 2, 1> diff =
          mePos - aPos;           // Calculate the difference in position between the robot and the agent
      if (diff.norm() < (T)1e-6)  // If the robot and agent are at the same position
      {
        diff = Eigen::Matrix<T, 2, 1>((T)1e-6, (T)0.0);  // Use a fixed small direction
      }
      Eigen::Matrix<T, 2, 1> diffDirection = diff.normalized();  // Normalize the difference vector

      Eigen::Matrix<T, 2, 1> aVel(agents(4, i) * ceres::cos(agents(2, i)),
                                  agents(4, i) * ceres::sin(agents(2, i)));  // Extract the velocity of the agent
      Eigen::Matrix<T, 2, 1> velDiff =
          meVel - aVel;  // Calculate the difference in velocity between the robot and the agent
      Eigen::Matrix<T, 2, 1> interactionVector =
          (T)sfm_lambda_ * velDiff + diffDirection;  // Calculate the interaction vector

      T interactionLength = interactionVector.norm();  // Calculate the length of the interaction vector
      Eigen::Matrix<T, 2, 1> interactionDirection =
          interactionVector / interactionLength;  // Normalize the interaction vector

      T theta = wrapToPi(ceres::atan2(diffDirection[1], diffDirection[0]) -
                         ceres::atan2(interactionDirection[1],
                                      interactionDirection[0]));  // Calculate the angle between the difference
                                                                  // direction and the interaction direction

      T B = (T)sfm_gamma_ *
            interactionLength;  // Calculate the social force parameter B based on the interaction length and gamma
      T forceVelocityAmount = -(T)ceres::exp(
          -(T)diff.norm() / B -
          ((T)sfm_nPrime_ * B * theta) * ((T)sfm_nPrime_ * B * theta));  // Calculate the force velocity amount based on
                                                                         // the difference in position and the angle

      T sign = (theta > (T)0) ? (T)1 : (T)-1;  // Determine the sign of theta

      T forceAngleAmount =
          -sign * ceres::exp(-(T)diff.norm() / B -
                             ((T)sfm_n_ * B * theta) *
                                 ((T)sfm_n_ * B * theta));  // Calculate the force angle amount based on the difference
                                                            // in position, the angle, and the sign of the initial angle

      Eigen::Matrix<T, 2, 1> forceVelocity =
          forceVelocityAmount * interactionDirection;  // Calculate the force velocity vector
      Eigen::Matrix<T, 2, 1> leftNormalVector(-interactionDirection[1],
                                              interactionDirection[0]);         // Calculate the left normal vector
      Eigen::Matrix<T, 2, 1> forceAngle = forceAngleAmount * leftNormalVector;  // Calculate the force angle vector

      meSocialforce += (T)sfm_forceFactorSocial_ * (forceVelocity + forceAngle);  // Accumulate the social force
    }

    return meSocialforce;
  }

private:
  double weight_;
  Eigen::Matrix<double, 6, 3> original_agents_;
  geometry_msgs::msg::Pose robot_init_;
  double counter_;
  unsigned int current_position_;
  double time_step_;
  unsigned int control_horizon_;
  unsigned int block_length_;
  double sfm_lambda_;
  double sfm_gamma_;
  double sfm_nPrime_;
  double sfm_n_;
  double sfm_relaxationTime_;
  double sfm_forceFactorSocial_;
};

}  // namespace nav2_social_mpc_controller

#endif