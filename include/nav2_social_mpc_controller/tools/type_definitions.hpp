#ifndef NAV2_SOCIAL_MPC_CONTROLLER__TYPE_DEFINITIONS_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__TYPE_DEFINITIONS_HPP_

#include <Eigen/Core>

typedef Eigen::Matrix<double, 6, 1> AgentStatus;       // x, y, yaw, timestamp, lv, av
typedef std::vector<AgentStatus> AgentsStates;         // vector of agent status (different agents at the same time)
typedef std::vector<AgentStatus> AgentTrajectory;      // vector of agent status (for a single agent trajectory)
typedef std::vector<AgentsStates> AgentsTrajectories;  // vector of agent states (trajectories for all agents)

#endif  // NAV2_SOCIAL_MPC_CONTROLLER__TYPE_DEFINITIONS_HPP_