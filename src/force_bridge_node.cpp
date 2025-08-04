#include "force_bridge_node.hpp"


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace force_bridge
{

ForceBridgeNode::ForceBridgeNode()
: Node("force_bridge_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/cartesian_compliance_controller/ft_sensor_wrench",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  wrench_pub_test_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "/gravitational_wrench",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  state_sub_ = this->create_subscription<franka_msgs::msg::FrankaState>(
    "/franka_robot_state_broadcaster/robot_state", 10,
    std::bind(&ForceBridgeNode::stateCallback, this, std::placeholders::_1));


  // Use parameters for mass and center of mass (com)
  this->declare_parameter<double>("mass", 0.8);
  this->declare_parameter<double>("com_x", 0.0875);
  this->declare_parameter<double>("com_y", -0.075);
  this->declare_parameter<double>("com_z", 0.0);
  mass_ = this->get_parameter("mass").as_double();
  com_.x = this->get_parameter("com_x").as_double();
  com_.y = this->get_parameter("com_y").as_double();
  com_.z = this->get_parameter("com_z").as_double();

  RCLCPP_INFO(this->get_logger(), "ForceBridgeNode started.");
}

void ForceBridgeNode::stateCallback(const franka_msgs::msg::FrankaState::SharedPtr msg)
{

  // Compute gravitational wrench in panda_link8 frame
  auto gravitational_wrench = computeWrench(
    tf_buffer_, 
    "panda_link8", 
    "panda_link0", 
    mass_, 
    com_);
  gravitational_wrench.header.stamp = msg->header.stamp;
  gravitational_wrench.header.frame_id = "panda_link8";

  // The wrench is computed as the negative of the external forces minus the gravitational wrench
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = msg->header.stamp;
  wrench_msg.header.frame_id = "panda_link8";
  wrench_msg.wrench.force.x  = -msg->k_f_ext_hat_k[0] - gravitational_wrench.wrench.force.x;
  wrench_msg.wrench.force.y  = -msg->k_f_ext_hat_k[1] - gravitational_wrench.wrench.force.y;
  wrench_msg.wrench.force.z  = -msg->k_f_ext_hat_k[2] - gravitational_wrench.wrench.force.z;
  wrench_msg.wrench.torque.x = -msg->k_f_ext_hat_k[3] - gravitational_wrench.wrench.torque.x;
  wrench_msg.wrench.torque.y = -msg->k_f_ext_hat_k[4] - gravitational_wrench.wrench.torque.y;
  wrench_msg.wrench.torque.z = -msg->k_f_ext_hat_k[5] - gravitational_wrench.wrench.torque.z;

  // Publish the wrench message
  try {
    auto wrench_transformed = tf_buffer_.transform(
      wrench_msg, "panda_link8", tf2::durationFromSec(0.0));
    wrench_pub_->publish(wrench_transformed);
    wrench_pub_test_->publish(gravitational_wrench);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
  }
}

geometry_msgs::msg::WrenchStamped ForceBridgeNode::computeWrench(
    const tf2_ros::Buffer& tf_buffer,
    const std::string& frame_to,    // "panda_link8"
    const std::string& frame_from,  // "panda_link0"
    double mass,
    const geometry_msgs::msg::Vector3& com
)
{
    geometry_msgs::msg::WrenchStamped wrench_out;

    // 1) compute gravitational force in world frame
    tf2::Vector3 F_world(0, 0, -9.81 * mass);

    try {
        // 2) get the transformation from frame_from to frame_to
        auto tf_stamped = tf_buffer.lookupTransform(
            frame_to, frame_from, tf2::TimePointZero);

        tf2::Quaternion q(
            tf_stamped.transform.rotation.x,
            tf_stamped.transform.rotation.y,
            tf_stamped.transform.rotation.z,
            tf_stamped.transform.rotation.w);
        tf2::Matrix3x3 R(q);

        // 3) express the force in the frame_to
        tf2::Vector3 F_r8 = R * F_world;
        wrench_out.wrench.force.x = F_r8.x();
        wrench_out.wrench.force.y = F_r8.y();
        wrench_out.wrench.force.z = F_r8.z();

        // 4) compute torque r × F
        tf2::Vector3 r(com.x, com.y, com.z);
        tf2::Vector3 M = r.cross(F_r8);
        wrench_out.wrench.torque.x = M.x();
        wrench_out.wrench.torque.y = M.y();
        wrench_out.wrench.torque.z = M.z();

    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(
          rclcpp::get_logger("computeWrench"),
          "Could not transform: %s", ex.what());
        // en cas d'erreur, on renvoie zéro
        wrench_out.wrench.force.x = 0;
        wrench_out.wrench.force.y = 0;
        wrench_out.wrench.force.z = 0;
        wrench_out.wrench.torque.x = 0;
        wrench_out.wrench.torque.y = 0;
        wrench_out.wrench.torque.z = 0;
    }
    return wrench_out;
}

}  // namespace force_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<force_bridge::ForceBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}