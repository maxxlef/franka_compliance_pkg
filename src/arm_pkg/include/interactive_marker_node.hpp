/**
 * @file interactive_marker_node.hpp
 * @brief Definition of the InteractiveMarkerNode class for managing an interactive RViz marker.
 * @note Intended to be used in RViz for manual control/debugging.
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_control.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @brief Node for creating and managing an interactive marker in ROS.
 * This node allows users to interactively set a target pose for a robotic arm using 6-DOF controls.
 * It publishes the target pose and broadcasts the corresponding TF transform.
 * The target pose is initialized from the TF transform between "panda_link0" and "panda_link8".
 */
class InteractiveMarkerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the InteractiveMarkerNode.
   * Initializes the interactive marker server, publishers, and TF listener.
   */
  InteractiveMarkerNode();

private:
  /**
   * @brief Try to initialize the target pose from the TF transform between "panda_link0" and "panda_link8".
   */
  void tryInitializePoseFromTF();
  /**
   * @brief Create the interactive marker and add controls to it.
   */
  void makeInteractiveMarker();
  /**
   * @brief Add 6-DOF controls to the interactive marker.
   * @param int_marker The interactive marker to which controls will be added.
   */
  void add6DofControls(visualization_msgs::msg::InteractiveMarker &int_marker);
  /**
   * @brief Process feedback from the interactive marker.
   * @param feedback The feedback message containing the updated pose.
   */
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
  /**
   * @brief Timer callback to publish the current pose and broadcast the TF transform.
   */
  void onTimer();
  
  /// Interactive marker server to manage the interactive marker in Rviz on the "interactive_marker_server" topic
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  /// Publisher for the target pose on the "cartesian_compliance_controller/target_frame" topic
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Pose current_pose_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};
