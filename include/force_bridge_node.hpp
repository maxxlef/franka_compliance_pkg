/**
 * @file force_bridge_node.hpp
 * @brief This file defines the ForceBridgeNode class for compensating gravitational forces
 * applied at the end-effector of a robotic arm and publishing the compensated wrench for compliance control.
 */

 #ifndef ARM_COMPLIANCE_CONTROL__FORCE_BRIDGE_NODE_HPP_
 #define ARM_COMPLIANCE_CONTROL__FORCE_BRIDGE_NODE_HPP_
 
 #include <rclcpp/rclcpp.hpp>
 #include <geometry_msgs/msg/wrench_stamped.hpp>
 #include <geometry_msgs/msg/vector3.hpp>
 #include <tf2_ros/transform_listener.h>
 #include <tf2_ros/buffer.h>
 #include <franka_msgs/msg/franka_state.hpp>
 #include <string>
 
 namespace force_bridge
 {
 
/**
 * @brief Node that compensates for gravitational forces applied at the end-effector.
 *
 * This node subscribes to the robot's Franka state and uses TF to retrieve the transform of the end-effector
 * (typically `panda_link8`). It computes the external wrench in the end-effector frame and subtracts the
 * expected gravitational wrench, based on the configured mass and center of mass of the tool.
 *
 * The compensated wrench is then published in the `panda_link8` frame, and can be used by a compliance controller.
 * 
 * @note Mass and center of mass can be configured via `config/params.yaml`.
 */
 class ForceBridgeNode : public rclcpp::Node
 {
 public:
   /**
    * @brief Constructor for ForceBridgeNode.
    * Initializes subscribers, publishers, and TF listener.
    */
   ForceBridgeNode();
 
 private:
   /**
    * @brief Callback for Franka state updates.
    * @param msg The latest Franka state message.
    */
   void stateCallback(const franka_msgs::msg::FrankaState::SharedPtr msg);
 
   /// Mass of the end-effector (set from parameters or fixed)
   double mass_;
 
   /// Center of mass of the end-effector expressed in its local frame
   geometry_msgs::msg::Vector3 com_;
 
   /**
    * @brief Computes the expected gravitational wrench given a transform between two frames.
    * @param tf_buffer TF buffer used for lookup.
    * @param frame_to Target frame in which the wrench should be expressed.
    * @param frame_from The reference frame used to express gravity (e.g., the world or base frame).
    * @param mass Mass of all components mounted on the end-effector.
    * @param com Center of mass vector.
    * @return Computed WrenchStamped representing the compensated wrench.
    */
   geometry_msgs::msg::WrenchStamped computeWrench(
     const tf2_ros::Buffer& tf_buffer,
     const std::string& frame_to,
     const std::string& frame_from,
     double mass,
     const geometry_msgs::msg::Vector3& com);
 
   // --- ROS interfaces ---
 
   /// Publisher for the computed gravitational wrench (for use in compliance control)
   rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
 
   /// (Optional) Publisher for testing or debugging purposes
   rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_test_;
 
   /// Subscription to the robot's Franka state
   rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr state_sub_;
 
   // --- TF infrastructure ---
 
   /// Buffer to store TF transformations
   tf2_ros::Buffer tf_buffer_;
 
   /// Listener attached to the TF buffer to receive transforms
   tf2_ros::TransformListener tf_listener_;
 };
 
 }  // namespace force_bridge
 
 #endif  // ARM_COMPLIANCE_CONTROL__FORCE_BRIDGE_NODE_HPP_
 