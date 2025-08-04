#ifndef WRENCH_SERVICE_NODE_HPP_
#define WRENCH_SERVICE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace wrench_checker
{

class WrenchServiceNode : public rclcpp::Node
{
public:
  WrenchServiceNode();

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void handle_service(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;


  double z_force_threshold_;
  double latest_force_z_;
  double latest_force_z_world_;
};

}  // namespace wrench_checker

#endif  // WRENCH_SERVICE_NODE_HPP_
