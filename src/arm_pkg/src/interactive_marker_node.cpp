#include "interactive_marker_node.hpp"

using namespace std::chrono_literals;

InteractiveMarkerNode::InteractiveMarkerNode()
: Node("interactive_marker_node")
{
  pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "target_arm", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "interactive_marker_server", this);

  init_timer_ = this->create_wall_timer(
    500ms, std::bind(&InteractiveMarkerNode::tryInitializePoseFromTF, this));
}

void InteractiveMarkerNode::tryInitializePoseFromTF()
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform("panda_link0", "panda_link8", tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for TF: %s", ex.what());
    return;
  }

  current_pose_.position.x = tf.transform.translation.x;
  current_pose_.position.y = tf.transform.translation.y;
  current_pose_.position.z = tf.transform.translation.z;
  current_pose_.orientation = tf.transform.rotation;

  RCLCPP_INFO(this->get_logger(), "Target initialized to panda_link8 position.");

  makeInteractiveMarker();
  server_->applyChanges();

  timer_ = this->create_wall_timer(
    50ms, std::bind(&InteractiveMarkerNode::onTimer, this));

  init_timer_->cancel();
}

void InteractiveMarkerNode::makeInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "panda_link0";
  int_marker.name = "target_marker";
  int_marker.description = "6-DOF Target Pose";
  int_marker.pose = current_pose_;
  int_marker.scale = 0.2;

  add6DofControls(int_marker);

  server_->insert(int_marker,
    std::bind(&InteractiveMarkerNode::processFeedback, this, std::placeholders::_1));
}

void InteractiveMarkerNode::add6DofControls(visualization_msgs::msg::InteractiveMarker &int_marker)
{
  using visualization_msgs::msg::InteractiveMarkerControl;
  InteractiveMarkerControl control;

  std::vector<std::tuple<std::string, float, float, float>> controls = {
    {"move_x", 1, 0, 0}, {"move_y", 0, 1, 0}, {"move_z", 0, 0, 1},
    {"rotate_x", 1, 0, 0}, {"rotate_y", 0, 1, 0}, {"rotate_z", 0, 0, 1}
  };

  for (const auto &[name, x, y, z] : controls) {
    control.orientation.w = 1;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.name = name;
    control.interaction_mode = name.find("move") != std::string::npos ?
      InteractiveMarkerControl::MOVE_AXIS : InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
  }
}

void InteractiveMarkerNode::processFeedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE) {
    current_pose_ = feedback->pose;
    server_->setPose(feedback->marker_name, current_pose_);
    server_->applyChanges();
  }
}

void InteractiveMarkerNode::onTimer()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "panda_link0";
  msg.pose = current_pose_;
  pose_pub_->publish(msg);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = "panda_link0";
  tf.child_frame_id = "target_marker";
  tf.transform.translation.x = current_pose_.position.x;
  tf.transform.translation.y = current_pose_.position.y;
  tf.transform.translation.z = current_pose_.position.z;
  tf.transform.rotation = current_pose_.orientation;
  tf_broadcaster_->sendTransform(tf);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InteractiveMarkerNode>());
  rclcpp::shutdown();
  return 0;
}
