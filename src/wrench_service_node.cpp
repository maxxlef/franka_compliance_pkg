#include "wrench_service_node.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wrench_checker
{

WrenchServiceNode::WrenchServiceNode()
: Node("wrench_service_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  latest_force_z_world_(0.0)
{
  // Paramètre de seuil
  this->declare_parameter<double>("z_force_threshold", 1.0);
  z_force_threshold_ = this->get_parameter("z_force_threshold").as_double();

  // Souscription au wrench en frame "panda_link8"
  wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/cartesian_compliance_controller/ft_sensor_wrench", 10,
    std::bind(&WrenchServiceNode::wrench_callback, this, std::placeholders::_1));

  // Service Trigger
  service_ = this->create_service<std_srvs::srv::Trigger>(
    "check_force_threshold",
    std::bind(&WrenchServiceNode::handle_service, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "WrenchServiceNode started.");
}

void WrenchServiceNode::wrench_callback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  // 1) On transforme le wrench dans la frame "world" (ou "panda_link0")
  geometry_msgs::msg::WrenchStamped wrench_world;
  try {
    // Change "world" ci‑dessous pour le frame de référence qui t’intéresse
    wrench_world = tf_buffer_.transform(
      *msg, 
      "panda_link0",      // target frame
      tf2::durationFromSec(0.1)
    );

    // 2) On stocke la composante Z du wrench monde
    latest_force_z_world_ = wrench_world.wrench.force.z;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Wrench Z in world frame: %.2f", latest_force_z_world_);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(),
                "TF transform failed in wrench_callback: %s", ex.what());
    // On garde la dernière valeur stockée, ou 0 si jamais initialisée
  }
}

void WrenchServiceNode::handle_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  bool exceeded = (std::abs(latest_force_z_world_) > z_force_threshold_);
  resp->success = exceeded;
  std::ostringstream ss;
  ss << "Force Z world = " << latest_force_z_world_
     << ", threshold = " << z_force_threshold_;
  resp->message = ss.str();
}

}  // namespace wrench_checker

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<wrench_checker::WrenchServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
