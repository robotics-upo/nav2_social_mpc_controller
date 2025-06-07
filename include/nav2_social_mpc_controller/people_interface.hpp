#ifndef NAV2_SOCIAL_MPC_CONTROLLER__PEOPLE_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__PEOPLE_HPP_

#include <mutex>

#include "people_msgs/msg/people.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_social_mpc_controller
{

class PeopleInterface
{
public:
  PeopleInterface(rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ~PeopleInterface();

  people_msgs::msg::People getPeople();

  void people_callback(const people_msgs::msg::People::SharedPtr people);

private:
  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  people_msgs::msg::People people_;
  std::mutex mutex_;
};
}  // namespace nav2_social_mpc_controller

#endif