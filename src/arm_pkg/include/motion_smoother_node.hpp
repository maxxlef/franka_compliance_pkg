#ifndef MOTION_SMOOTHER_NODE_HPP_
#define MOTION_SMOOTHER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class MotionSmootherNode : public rclcpp::Node
{
public:
  MotionSmootherNode();

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void inputTimerCallback();
  void interpolateAndPublish();

  // Parameters
  bool use_tf_input_;
  std::string input_topic_;
  std::string input_frame_;
  std::string output_topic_;
  double publish_rate_;
  double step_distance_;
  double timeout_sec_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr input_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // State
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::PoseStamped target_pose_;
  bool new_target_ = false;
  rclcpp::Time last_input_time_;


  // Thread safety
  std::mutex mutex_;
};

#endif  // MOTION_SMOOTHER_NODE_HPP_
