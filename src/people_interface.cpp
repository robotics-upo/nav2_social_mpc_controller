#include "nav2_social_mpc_controller/people_interface.hpp"

namespace nav2_social_mpc_controller
{

PeopleInterface::PeopleInterface(rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  auto node = parent.lock();
  people_sub_ = node->create_subscription<people_msgs::msg::People>(
    "people", rclcpp::SensorDataQoS(),
    std::bind(&PeopleInterface::people_callback, this, std::placeholders::_1));
}

PeopleInterface::~PeopleInterface() {}

void PeopleInterface::people_callback(const people_msgs::msg::People::SharedPtr people)
{
  mutex_.lock();
  people_ = *people;
  mutex_.unlock();
}

people_msgs::msg::People PeopleInterface::getPeople()
{
  mutex_.lock();
  people_msgs::msg::People p = people_;
  mutex_.unlock();
  return p;
}

}  // namespace nav2_social_mpc_controller
