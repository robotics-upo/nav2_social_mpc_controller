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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "nav2_path_trajectorizer/path_trajectorizer.hpp"

namespace nav2_path_trajectorizer
{

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // std::shared_ptr<rclcpp::Node> node =
  //    rclcpp::Node::make_shared("test_path_trajectorizer");

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_lifecycle_;
  std::shared_ptr<nav2_path_trajectorizer::PathTrajectorizer> smoother_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  node_lifecycle_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "PathTrajectorizerTestNode", rclcpp::NodeOptions());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_lifecycle_->get_clock());

  node_lifecycle_->configure();
  node_lifecycle_->activate();

  // Create the trajectorizer
  smoother_ = std::make_shared<nav2_path_trajectorizer::PathTrajectorizer>();

  smoother_->configure(node_lifecycle_, "PathTrajectorizer", tf_buffer_, nullptr, nullptr);

  smoother_->activate();

  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.omnidirectional", false));
  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.desired_linear_vel", 0.4));
  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.lookahead_dist", 0.4));
  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.max_angular_vel", 1.0));
  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.transform_tolerance", 0.1));
  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.base_frame", "base_link"));
  node_lifecycle_->set_parameter(rclcpp::Parameter("PathTrajectorizer.time_step", 0.01));

  // Create a path
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = node_lifecycle_->get_clock()->now();
  geometry_msgs::msg::PoseStamped p;
  p.header = path.header;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, 0);
  p.pose.orientation = tf2::toMsg(myQuaternion);
  // generate path as senoidal signal
  n = 20;
  step = 0.1;
  double a = 1.0;
  double b = 1.0;
  double x[n];
  double y[n];
  for (unsigned int i = 0; i < n; i++) {
    if (i == 0)
      x[i] = 0.0;
    else
      x[i] = x[i - 1] + step;
    y[i] = a * sin(x[i]) + b;
  }
  y[0] = 0.0;
  for (unsigned int i = 0; i < n; i++) {
    p.pose.position.x = x[i];
    p.pose.position.y = y[i];
    path.poses.push_back(p);
  }

  rclcpp::Rate r(1);  // Hz
  for (unsigned int i = 0; i < 20; i++) {
    // call the smoother
    smoother_->smooth(path, 2.0);
    r.sleep();
  }

  // end
  smoother_->deactivate();
  smoother_->cleanup();
  node_lifecycle_->deactivate();
}
