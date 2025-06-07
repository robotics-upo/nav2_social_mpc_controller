#ifndef NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_INTERFACE_HPP_
#define NAV2_SOCIAL_MPC_CONTROLLER__OBSTACLE_INTERFACE_HPP_

#include <mutex>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "obstacle_distance_msgs/msg/obstacle_distance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"

namespace nav2_social_mpc_controller
{

class ObstacleDistInterface
{
public:
  ObstacleDistInterface(rclcpp_lifecycle::LifecycleNode::WeakPtr parent, const std::string& agents_frame_id,
                        std::shared_ptr<tf2_ros::Buffer> tf_buffer, tf2::Duration transform_tolerance);
  ~ObstacleDistInterface();

  /**
   * @brief Get the obstacle distance msg in the agents frame ID
   * @return obstacle_distance_msgs::msg::ObstacleDistance
   */
  obstacle_distance_msgs::msg::ObstacleDistance getDistanceTransform();

  /**
   * @brief Get the origin pose stamped from the obstacle distance msg
   * @return geometry_msgs::msg::PoseStamped
   */
  geometry_msgs::msg::PoseStamped getOriginPoseStamped(const obstacle_distance_msgs::msg::ObstacleDistance& od);

  /**
   * @brief Get the agents frame ID
   * @return std::string
   */
  inline std::string getAgentsFrameId() const
  {
    return agents_frame_id_;
  }
  /**
   * @brief Set the agents frame ID
   * @param agents_frame_id The agents frame ID
   */
  inline void setAgentsFrameId(const std::string& agents_frame_id)
  {
    agents_frame_id_ = agents_frame_id;
  }

  /**
   * @brief Transform a world point to a cell in the obstacle distance msg
   * @param p World point
   * @param i Cell index in x direction
   * @param j Cell index in y direction
   */
  void worldpoint2Cell(const geometry_msgs::msg::Point& p, unsigned int& i, unsigned int& j);

  /**
   * @brief Transform a cell in the obstacle distance msg to a world point
   * @param i Cell index in x direction
   * @param j Cell index in y direction
   * @param p World point
   */
  void cell2Worldpoint(const unsigned int& i, const unsigned int& j, geometry_msgs::msg::Point& p);

  /**
   * @brief Transform a cell in the obstacle distance msg to an index
   * @param i Cell index in x direction
   * @param j Cell index in y direction
   * @param index Index in the obstacle distance msg array
   * @return bool if successful
   */
  bool cell2Index(const unsigned int& i, const unsigned int& j, unsigned int& index);

  /**
   * @brief Transform an index in the obstacle distance msg array to a cell
   * @param index Index in the obstacle distance msg array
   * @param i Cell index in x direction
   * @param j Cell index in y direction
   * @return bool if successful
   */
  bool index2Cell(const unsigned int& index, unsigned int& i, unsigned int& j);

private:
  /**
   * @brief Transform the obstacle distance msg to the given frame ID
   * @param out_frame_id The frame ID to transform to
   * @param od The obstacle distance msg to transform
   * @return bool if successful
   */
  bool transformObstacleDistance(const std::string& out_frame_id, obstacle_distance_msgs::msg::ObstacleDistance& od);

  /**
   * @brief   Transform a point to another frame.
   * @param frame  Frame ID to transform to
   * @param in_point  Point input to transform
   * @param out_point Transformed output
   * @return bool if transformation is successful
   */
  bool transformPointStamped(const std::string frame, const geometry_msgs::msg::PointStamped& in_point,
                             geometry_msgs::msg::PointStamped& out_point) const;

  /**
   * @brief   Transform a pose to another frame.

    * @param frame Frame ID to transform to
    * @param in_pose Pose input to transform
    * @param out_pose Transformed output

    * @return bool if transformation is successful
    */
  bool transformPose(const std::string frame, const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose) const;
  /**
   * @brief This callback is called when a new obstacle distance msg is received.
   * @param msg ObstacleDistance msg
   */
  void obs_callback(const obstacle_distance_msgs::msg::ObstacleDistance::SharedPtr msg);

  rclcpp::Subscription<obstacle_distance_msgs::msg::ObstacleDistance>::ConstSharedPtr obs_sub_;
  obstacle_distance_msgs::msg::ObstacleDistance obs_;
  std::mutex mutex_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2::Duration transform_tolerance_;
  std::string agents_frame_id_ = "map";  // Default frame ID for agents

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent;
};
}  // namespace nav2_social_mpc_controller

#endif