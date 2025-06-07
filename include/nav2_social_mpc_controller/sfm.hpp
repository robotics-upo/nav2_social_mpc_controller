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

#ifndef NAV2_SOCIAL_MPC_CONTROLLER__SFM_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__SFM_HPP_

#include <cmath>
#include <list>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"

namespace sfm_controller
{

struct Forces
{
  Eigen::Vector2d desiredForce;
  Eigen::Vector2d obstacleForce;
  Eigen::Vector2d socialForce;
  Eigen::Vector2d groupGazeForce;
  Eigen::Vector2d groupCoherenceForce;
  Eigen::Vector2d groupRepulsionForce;
  Eigen::Vector2d groupForce;
  Eigen::Vector2d globalForce;
  Eigen::Vector2d robotSocialForce;
};

struct Parameters
{
  Parameters()
  : forceFactorDesired(2.0),
    forceFactorObstacle(20),  // forceFactorObstacle(10),
    forceSigmaObstacle(0.2),
    forceFactorSocial(2.1),
    forceFactorGroupGaze(3.0),
    forceFactorGroupCoherence(2.0),
    forceFactorGroupRepulsion(1.0),
    lambda(2.0),
    gamma(0.35),
    n(2.0),
    nPrime(3.0),
    relaxationTime(0.5)
  {
  }

  double forceFactorDesired;
  double forceFactorObstacle;
  double forceSigmaObstacle;
  double forceFactorSocial;
  double forceFactorGroupGaze;
  double forceFactorGroupCoherence;
  double forceFactorGroupRepulsion;
  double lambda;
  double gamma;
  double n;
  double nPrime;
  double relaxationTime;
};

struct Goal
{
  Eigen::Vector2d center;
  double radius;
};

// struct closest_obs {
//   Eigen::Vector2d direction;
//   double distance;
// };

struct Agent
{
  Agent()
  : groupId(-1),
    desiredVelocity(0.6),
    radius(0.35),
    cyclicGoals(false),
    linearVelocity(0),
    angularVelocity(0)
  {
  }

  Agent(double linearVelocity, double angularVelocity)
  : groupId(-1),
    desiredVelocity(0.6),
    radius(0.35),
    cyclicGoals(false),
    linearVelocity(linearVelocity),
    angularVelocity(angularVelocity)
  {
  }

  Agent(
    const Eigen::Vector2d & position, const double & yaw, double linearVelocity,
    double angularVelocity)
  : groupId(-1),
    position(position),
    yaw(yaw),
    desiredVelocity(0.6),
    radius(0.35),
    cyclicGoals(false),
    linearVelocity(linearVelocity),
    angularVelocity(angularVelocity)
  {
  }

  int id;
  int groupId;

  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  // utils::Angle yaw;
  double yaw;
  // Eigen::Vector2d movement;

  double desiredVelocity;
  double radius;

  std::list<Goal> goals;
  bool cyclicGoals;

  double linearVelocity;
  double angularVelocity;

  Forces forces;
  Parameters params;
  // obstacles1 contains the positions (x,y) of the obstacles
  std::vector<Eigen::Vector2d> obstacles1;
  // obstacles2 contains the vectors between the agent
  // and the obstacle
  std::vector<Eigen::Vector2d> obstacles2;
};

struct Group
{
  Eigen::Vector2d center;
  std::vector<unsigned> agents;
};

class SocialForceModel
{
public:
  SocialForceModel(SocialForceModel const &) = delete;
  void operator=(SocialForceModel const &) = delete;
  ~SocialForceModel() {}

  static SocialForceModel & getInstance()
  {
    static SocialForceModel singleton;
    return singleton;
  }

#define SFM SocialForceModel::getInstance()

  std::vector<Agent> & computeForces(std::vector<Agent> & agents) const;
  void computeForces(Agent & me, std::vector<Agent> & agents);
  std::vector<Agent> & updatePosition(std::vector<Agent> & agents, double dt) const;
  void updatePosition(Agent & me, double dt) const;

private:
#define PW(x) ((x) * (x))
  SocialForceModel() {}
  Eigen::Vector2d computeDesiredForce(Agent & agent) const;
  void computeObstacleForce(Agent & agent) const;
  void computeSocialForce(unsigned index, std::vector<Agent> & agents) const;
  void computeSocialForce(Agent & agent, std::vector<Agent> & agents) const;
  void computeGroupForce(
    unsigned index, const Eigen::Vector2d & desiredDirection, std::vector<Agent> & agents,
    const std::unordered_map<int, Group> & groups) const;
  void computeGroupForce(
    Agent & me, const Eigen::Vector2d & desiredDirection, std::vector<Agent> & agents,
    Group & group) const;
};

inline Eigen::Vector2d SocialForceModel::computeDesiredForce(Agent & agent) const
{
  Eigen::Vector2d desiredDirection;
  if (
    !agent.goals.empty() &&
    (agent.goals.front().center - agent.position).norm() > agent.goals.front().radius) {
    Eigen::Vector2d diff = agent.goals.front().center - agent.position;
    desiredDirection = diff.normalized();
    agent.forces.desiredForce = agent.params.forceFactorDesired *
                                (desiredDirection * agent.desiredVelocity - agent.velocity) /
                                agent.params.relaxationTime;
  } else {
    agent.forces.desiredForce = -agent.velocity / agent.params.relaxationTime;
  }
  return desiredDirection;
}

inline void SocialForceModel::computeObstacleForce(Agent & agent) const
{
  if (agent.obstacles1.size() > 0 || agent.obstacles2.size() > 0) {
    agent.forces.obstacleForce << 0, 0;
    for (unsigned i = 0; i < agent.obstacles1.size(); i++) {
      Eigen::Vector2d minDiff = agent.position - agent.obstacles1[i];
      double distance = minDiff.norm() - agent.radius;
      agent.forces.obstacleForce += agent.params.forceFactorObstacle *
                                    std::exp(-distance / agent.params.forceSigmaObstacle) *
                                    minDiff.normalized();
    }
    for (unsigned i = 0; i < agent.obstacles2.size(); i++) {
      Eigen::Vector2d minDiff = agent.obstacles2[i];
      double distance = minDiff.norm() - agent.radius;
      agent.forces.obstacleForce += agent.params.forceFactorObstacle *
                                    std::exp(-distance / agent.params.forceSigmaObstacle) *
                                    minDiff.normalized();
    }
    agent.forces.obstacleForce /= (double)(agent.obstacles1.size() + agent.obstacles2.size());
    //   } else if (map != NULL) {
    //     const Map::Obstacle &obs = map->getNearestObstacle(agent.position);
    //     utils::Vector2d minDiff = agent.position - obs.position;
    //     double distance = minDiff.norm() - agent.radius;
    //     agent.forces.obstacleForce =
    //         agent.params.forceFactorObstacle *
    //         std::exp(-distance / agent.params.forceSigmaObstacle) *
    //         minDiff.normalized();
  } else {
    agent.forces.obstacleForce << 0, 0;
  }
}

inline void SocialForceModel::computeSocialForce(unsigned index, std::vector<Agent> & agents) const
{
  Agent & agent = agents[index];
  agent.forces.socialForce << 0, 0;
  for (unsigned i = 0; i < agents.size(); i++) {
    if (i == index) {
      continue;
    }
    Eigen::Vector2d diff = agents[i].position - agent.position;
    Eigen::Vector2d diffDirection = diff.normalized();
    Eigen::Vector2d velDiff = agent.velocity - agents[i].velocity;
    Eigen::Vector2d interactionVector = agent.params.lambda * velDiff + diffDirection;
    double interactionLength = interactionVector.norm();
    Eigen::Vector2d interactionDirection = interactionVector / interactionLength;
    double a1 = std::atan2(interactionDirection[1], interactionDirection[0]);
    while (a1 <= -M_PI) a1 += 2 * M_PI;
    while (a1 > M_PI) a1 -= 2 * M_PI;
    double a2 = std::atan2(diffDirection[1], diffDirection[0]);
    while (a2 <= -M_PI) a2 += 2 * M_PI;
    while (a2 > M_PI) a2 -= 2 * M_PI;
    // Eigen::Angle theta = interactionDirection.angleTo(diffDirection);
    double theta = a2 - a1;
    while (theta <= -M_PI) theta += 2 * M_PI;
    while (theta > M_PI) theta -= 2 * M_PI;
    double B = agent.params.gamma * interactionLength;
    double thetaRad = theta;
    double forceVelocityAmount =
      -std::exp(-diff.norm() / B - PW(agent.params.nPrime * B * thetaRad));
    double thetaSign = -1.0;
    if (theta == 0) {
      thetaSign = 0;
    } else if (theta > 0) {
      thetaSign = 1;
    }
    double forceAngleAmount =
      -thetaSign * std::exp(-diff.norm() / B - PW(agent.params.n * B * thetaRad));
    Eigen::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
    Eigen::Vector2d interDir_leftNormalVector(-interactionDirection[1], interactionDirection[0]);
    Eigen::Vector2d forceAngle = forceAngleAmount * interDir_leftNormalVector;
    agent.forces.socialForce += agent.params.forceFactorSocial * (forceVelocity + forceAngle);
    if (i == 0) {
      agent.forces.robotSocialForce = agent.params.forceFactorSocial * (forceVelocity + forceAngle);
    }
  }
}

inline void SocialForceModel::computeSocialForce(Agent & me, std::vector<Agent> & agents) const
{
  // Agent& agent = agents[index];
  me.forces.socialForce << 0, 0;
  for (unsigned i = 0; i < agents.size(); i++) {
    if (agents[i].id == me.id) {
      continue;
    }
    Eigen::Vector2d diff = agents[i].position - me.position;
    Eigen::Vector2d diffDirection = diff.normalized();
    Eigen::Vector2d velDiff = me.velocity - agents[i].velocity;
    Eigen::Vector2d interactionVector = me.params.lambda * velDiff + diffDirection;
    double interactionLength = interactionVector.norm();
    Eigen::Vector2d interactionDirection = interactionVector / interactionLength;
    double a1 = std::atan2(interactionDirection[1], interactionDirection[0]);
    while (a1 <= -M_PI) a1 += 2 * M_PI;
    while (a1 > M_PI) a1 -= 2 * M_PI;
    double a2 = std::atan2(diffDirection[1], diffDirection[0]);
    while (a2 <= -M_PI) a2 += 2 * M_PI;
    while (a2 > M_PI) a2 -= 2 * M_PI;
    // Eigen::Angle theta = interactionDirection.angleTo(diffDirection);
    double theta = a2 - a1;
    while (theta <= -M_PI) theta += 2 * M_PI;
    while (theta > M_PI) theta -= 2 * M_PI;
    double B = me.params.gamma * interactionLength;
    double thetaRad = theta;
    double forceVelocityAmount = -std::exp(-diff.norm() / B - PW(me.params.nPrime * B * thetaRad));
    double thetaSign = -1.0;
    if (theta == 0) {
      thetaSign = 0;
    } else if (theta > 0) {
      thetaSign = 1;
    }
    double forceAngleAmount =
      -thetaSign * std::exp(-diff.norm() / B - PW(me.params.n * B * thetaRad));
    Eigen::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
    Eigen::Vector2d interDir_leftNormalVector(-interactionDirection[1], interactionDirection[0]);
    Eigen::Vector2d forceAngle = forceAngleAmount * interDir_leftNormalVector;
    me.forces.socialForce += me.params.forceFactorSocial * (forceVelocity + forceAngle);
  }
}

inline void SocialForceModel::computeGroupForce(
  unsigned index, const Eigen::Vector2d & desiredDirection, std::vector<Agent> & agents,
  const std::unordered_map<int, Group> & groups) const
{
  Agent & agent = agents[index];
  agent.forces.groupForce << 0, 0;
  agent.forces.groupGazeForce << 0, 0;
  agent.forces.groupCoherenceForce << 0, 0;
  agent.forces.groupRepulsionForce << 0, 0;
  if (groups.count(agent.groupId) == 0 || groups.at(agent.groupId).agents.size() < 2) {
    return;
  }
  const Group & group = groups.at(agent.groupId);

  // Gaze force
  Eigen::Vector2d com = group.center;
  com = (1 / (double)(group.agents.size() - 1)) * (group.agents.size() * com - agent.position);

  Eigen::Vector2d relativeCom = com - agent.position;
  double visionAngle = M_PI_2;
  double elementProduct = desiredDirection.dot(relativeCom);
  double comAngle = std::acos(elementProduct / (desiredDirection.norm() * relativeCom.norm()));
  while (comAngle <= -M_PI) comAngle += 2 * M_PI;
  while (comAngle > M_PI) comAngle -= 2 * M_PI;
  if (comAngle > visionAngle) {
#ifdef _PAPER_VERSION_
    utils::Angle necessaryRotation = comAngle - visionAngle;
    agent.forces.groupGazeForce = -necessaryRotation.toRadian() * desiredDirection;
#else
    double desiredDirectionSquared = desiredDirection.squaredNorm();
    double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
    agent.forces.groupGazeForce = desiredDirectionDistance * desiredDirection;
#endif
    agent.forces.groupGazeForce *= agent.params.forceFactorGroupGaze;
  }

  // Coherence force
  com = group.center;
  relativeCom = com - agent.position;
  double distance = relativeCom.norm();
  double maxDistance = ((double)group.agents.size() - 1) / 2;
#ifdef _PAPER_VERSION_
  if (distance >= maxDistance) {
    agent.forces.groupCoherenceForce = relativeCom.normalized();
    agent.forces.groupCoherenceForce *= agent.params.forceFactorGroupCoherence;
  }
#else
  agent.forces.groupCoherenceForce = relativeCom;
  double softenedFactor =
    agent.params.forceFactorGroupCoherence * (std::tanh(distance - maxDistance) + 1) / 2;
  agent.forces.groupCoherenceForce *= softenedFactor;
#endif

  // Repulsion Force
  for (unsigned i = 0; i < group.agents.size(); i++) {
    if (index == group.agents[i]) {
      continue;
    }
    Eigen::Vector2d diff = agent.position - agents.at(group.agents[i]).position;
    if (diff.norm() < agent.radius + agents.at(group.agents[i]).radius) {
      agent.forces.groupRepulsionForce += diff;
    }
  }
  agent.forces.groupRepulsionForce *= agent.params.forceFactorGroupRepulsion;

  // Group Force
  agent.forces.groupForce = agent.forces.groupGazeForce + agent.forces.groupCoherenceForce +
                            agent.forces.groupRepulsionForce;
}

inline void SocialForceModel::computeGroupForce(
  Agent & me, const Eigen::Vector2d & desiredDirection, std::vector<Agent> & agents,
  Group & group) const
{
  // Agent& agent = agents[index];
  me.forces.groupForce << 0, 0;
  me.forces.groupGazeForce << 0, 0;
  me.forces.groupCoherenceForce << 0, 0;
  me.forces.groupRepulsionForce << 0, 0;
  if (group.agents.size() < 2) {
    return;
  }

  // Gaze force
  Eigen::Vector2d com = group.center;
  com = (1 / (double)(group.agents.size() - 1)) * (group.agents.size() * com - me.position);

  Eigen::Vector2d relativeCom = com - me.position;
  double visionAngle = M_PI_2;
  double elementProduct = desiredDirection.dot(relativeCom);
  double comAngle = std::acos(elementProduct / (desiredDirection.norm() * relativeCom.norm()));
  while (comAngle <= -M_PI) comAngle += 2 * M_PI;
  while (comAngle > M_PI) comAngle -= 2 * M_PI;
  if (comAngle > visionAngle) {
#ifdef _PAPER_VERSION_
    utils::Angle necessaryRotation = comAngle - visionAngle;
    me.forces.groupGazeForce = -necessaryRotation.toRadian() * desiredDirection;
#else
    double desiredDirectionSquared = desiredDirection.squaredNorm();
    double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
    me.forces.groupGazeForce = desiredDirectionDistance * desiredDirection;
#endif
    me.forces.groupGazeForce *= me.params.forceFactorGroupGaze;
  }

  // Coherence force
  com = group.center;
  relativeCom = com - me.position;
  double distance = relativeCom.norm();
  double maxDistance = ((double)group.agents.size() - 1) / 2;
#ifdef _PAPER_VERSION_
  if (distance >= maxDistance) {
    me.forces.groupCoherenceForce = relativeCom.normalized();
    me.forces.groupCoherenceForce *= me.params.forceFactorGroupCoherence;
  }
#else
  me.forces.groupCoherenceForce = relativeCom;
  double softenedFactor =
    me.params.forceFactorGroupCoherence * (std::tanh(distance - maxDistance) + 1) / 2;
  me.forces.groupCoherenceForce *= softenedFactor;
#endif

  // Repulsion Force
  // Index 0 -> me
  for (unsigned i = 1; i < group.agents.size(); i++) {
    Eigen::Vector2d diff = me.position - agents.at(group.agents[i]).position;
    if (diff.norm() < me.radius + agents.at(group.agents[i]).radius) {
      me.forces.groupRepulsionForce += diff;
    }
  }
  me.forces.groupRepulsionForce *= me.params.forceFactorGroupRepulsion;

  // Group Force
  me.forces.groupForce =
    me.forces.groupGazeForce + me.forces.groupCoherenceForce + me.forces.groupRepulsionForce;
}

inline std::vector<Agent> & SocialForceModel::computeForces(std::vector<Agent> & agents) const
{
  std::unordered_map<int, Group> groups;
  for (unsigned i = 0; i < agents.size(); i++) {
    if (agents[i].groupId < 0) {
      continue;
    }
    groups[agents[i].groupId].agents.push_back(i);
    groups[agents[i].groupId].center += agents[i].position;
  }
  for (auto it = groups.begin(); it != groups.end(); ++it) {
    it->second.center /= (double)(it->second.agents.size());
  }

  for (unsigned i = 0; i < agents.size(); i++) {
    Eigen::Vector2d desiredDirection = computeDesiredForce(agents[i]);
    computeObstacleForce(agents[i]);
    computeSocialForce(i, agents);
    computeGroupForce(i, desiredDirection, agents, groups);
    agents[i].forces.globalForce = agents[i].forces.desiredForce + agents[i].forces.socialForce +
                                   agents[i].forces.obstacleForce + agents[i].forces.groupForce;
  }
  return agents;
}

inline void SocialForceModel::computeForces(Agent & me, std::vector<Agent> & agents)
{
  // form the group
  Group mygroup;
  if (me.groupId != -1) {
    mygroup.agents.push_back(me.id);
    mygroup.center = me.position;
    for (unsigned i = 0; i < agents.size(); i++) {
      if (agents[i].id == me.id) {
        continue;
      }
      if (agents[i].groupId == me.groupId) {
        mygroup.agents.push_back(i);
        mygroup.center += agents[i].position;
      }
    }
    mygroup.center /= (double)mygroup.agents.size();
  }

  // Compute agent's forces
  Eigen::Vector2d desiredDirection = computeDesiredForce(me);
  computeObstacleForce(me);
  computeSocialForce(me, agents);
  computeGroupForce(me, desiredDirection, agents, mygroup);
  me.forces.globalForce =
    me.forces.desiredForce + me.forces.socialForce + me.forces.obstacleForce + me.forces.groupForce;
}

// inline void Agent::move(double dt) {
//   double imd = linearVelocity * dt;
//   utils::Vector2d inc(
//       imd * std::cos(yaw.toRadian() + angularVelocity * dt * 0.5),
//       imd * std::sin(yaw.toRadian() + angularVelocity * dt * 0.5));
//   yaw += utils::Angle::fromRadian(angularVelocity * dt);
//   position += inc;
//   velocity.set(linearVelocity * yaw.cos(), linearVelocity * yaw.sin());
// }

inline std::vector<Agent> & SocialForceModel::updatePosition(
  std::vector<Agent> & agents, double dt) const
{
  for (unsigned i = 0; i < agents.size(); i++) {
    // Eigen::Vector2d initPos = agents[i].position;
    // if (agents[i].teleoperated) {
    //   double imd = agents[i].linearVelocity * dt;
    //   utils::Vector2d inc(imd * std::cos(agents[i].yaw.toRadian() +
    //                                      agents[i].angularVelocity * dt *
    //                                      0.5),
    //                       imd * std::sin(agents[i].yaw.toRadian() +
    //                                      agents[i].angularVelocity * dt *
    //                                      0.5));
    //   agents[i].yaw += utils::Angle::fromRadian(agents[i].angularVelocity *
    //   dt); agents[i].position += inc;
    //   agents[i].velocity.set(agents[i].linearVelocity * agents[i].yaw.cos(),
    //                          agents[i].linearVelocity * agents[i].yaw.sin());
    // } else {
    agents[i].velocity += agents[i].forces.globalForce * dt;
    if (agents[i].velocity.norm() > agents[i].desiredVelocity) {
      agents[i].velocity.normalize();
      agents[i].velocity *= agents[i].desiredVelocity;
    }
    double initYaw = agents[i].yaw;
    double yaw = std::atan2(agents[i].velocity[1], agents[i].velocity[0]);
    while (yaw <= -M_PI) yaw += 2 * M_PI;
    while (yaw > M_PI) yaw -= 2 * M_PI;
    agents[i].yaw = yaw;
    double angle = (agents[i].yaw - initYaw);
    while (angle <= -M_PI) angle += 2 * M_PI;
    while (angle > M_PI) angle -= 2 * M_PI;
    agents[i].angularVelocity = angle / dt;

    agents[i].position += agents[i].velocity * dt;
    agents[i].linearVelocity = agents[i].velocity.norm();
    //}
    // agents[i].movement = agents[i].position - initPos;
    if (
      !agents[i].goals.empty() && (agents[i].goals.front().center - agents[i].position).norm() <=
                                    agents[i].goals.front().radius) {
      Goal g = agents[i].goals.front();
      agents[i].goals.pop_front();
      if (agents[i].cyclicGoals) {
        agents[i].goals.push_back(g);
      }
    }
  }
  return agents;
}

inline void SocialForceModel::updatePosition(Agent & agent, double dt) const
{
  // Eigen::Vector2d initPos = agent.position;
  double initYaw = agent.yaw;

  agent.velocity += agent.forces.globalForce * dt;
  if (agent.velocity.norm() > agent.desiredVelocity) {
    agent.velocity.normalize();
    agent.velocity *= agent.desiredVelocity;
  }
  double yaw = std::atan2(agent.velocity[1], agent.velocity[0]);
  while (yaw <= -M_PI) yaw += 2 * M_PI;
  while (yaw > M_PI) yaw -= 2 * M_PI;
  agent.yaw = yaw;
  double angle = (agent.yaw - initYaw);
  while (angle <= -M_PI) angle += 2 * M_PI;
  while (angle > M_PI) angle -= 2 * M_PI;
  agent.angularVelocity = angle / dt;

  agent.position += agent.velocity * dt;
  agent.linearVelocity = agent.velocity.norm();

  // agent.movement = agent.position - initPos;
  if (
    !agent.goals.empty() &&
    (agent.goals.front().center - agent.position).norm() <= agent.goals.front().radius) {
    Goal g = agent.goals.front();
    agent.goals.pop_front();
    if (agent.cyclicGoals) {
      agent.goals.push_back(g);
    }
  }
}

}  // namespace sfm_controller
#endif