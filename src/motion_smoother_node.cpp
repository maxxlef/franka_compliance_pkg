#include "motion_smoother_node.hpp"

MotionSmootherNode::MotionSmootherNode()
: Node("motion_smoother_node")
{
  // Params
  this->declare_parameter<bool>("use_tf_input", false);
  this->declare_parameter<std::string>("input_topic", "object_pose");
  this->declare_parameter<std::string>("input_frame", "object_frame");
  this->declare_parameter<std::string>("output_topic", "smoothed_target_pose");
  this->declare_parameter<double>("publish_rate", 10.0);
  this->declare_parameter<double>("step_distance", 0.01);
  this->declare_parameter<double>("timeout", 4.0);

  this->get_parameter("use_tf_input", use_tf_input_);
  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("input_frame", input_frame_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("publish_rate", publish_rate_);
  this->get_parameter("step_distance", step_distance_);
  this->get_parameter("timeout", timeout_sec_);

  // Subscriber or TF
  if (!use_tf_input_) {
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MotionSmootherNode::poseCallback, this, std::placeholders::_1));
  } else {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Publisher
  publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    output_topic_, rclcpp::QoS(10));

  // Timers
  double period = 1.0 / publish_rate_;
  input_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period),
    std::bind(&MotionSmootherNode::inputTimerCallback, this));

  last_input_time_ = this->now();  // pour éviter la soustraction de temps non compatibles


  RCLCPP_INFO(this->get_logger(), "MotionSmootherNode initialized");
}

void MotionSmootherNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  target_pose_ = *msg;
  current_pose_ = current_pose_.header.stamp.sec == 0 ? target_pose_ : current_pose_;
  last_input_time_ = this->now();
  new_target_ = true;
}

void MotionSmootherNode::inputTimerCallback()
{
  rclcpp::Time now = this->now();

  // Get input
  if (use_tf_input_) {
    try {
      geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
        "base_link", input_frame_, this->now(), rclcpp::Duration::from_seconds(0.1));

      geometry_msgs::msg::PoseStamped pose;
      pose.header = t.header;
      pose.pose.position.x = t.transform.translation.x;
      pose.pose.position.y = t.transform.translation.y;
      pose.pose.position.z = t.transform.translation.z;
      pose.pose.orientation = t.transform.rotation;
      poseCallback(std::make_shared<geometry_msgs::msg::PoseStamped>(pose));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "TF lookup failed: %s", ex.what());
    }
  }

  // Timeout
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if ((now - last_input_time_).seconds() > timeout_sec_) {
      RCLCPP_ERROR(this->get_logger(), "Input timeout (> %.2f s)", timeout_sec_);
      return;
    }
  }

  interpolateAndPublish();
}

void MotionSmootherNode::interpolateAndPublish()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!new_target_) return;

  auto &cp = current_pose_.pose.position;
  auto &tp = target_pose_.pose.position;
  double dx = tp.x - cp.x;
  double dy = tp.y - cp.y;
  double dz = tp.z - cp.z;
  double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

  if (dist < 1e-6) {
    new_target_ = false;
    return;
  }

  double step = std::min(step_distance_, dist);
  double ratio = step / dist;

  geometry_msgs::msg::PoseStamped next = current_pose_;
  next.pose.position.x += dx * ratio;
  next.pose.position.y += dy * ratio;
  next.pose.position.z += dz * ratio;

  tf2::Quaternion q_curr, q_target;
  tf2::fromMsg(current_pose_.pose.orientation, q_curr);
  tf2::fromMsg(target_pose_.pose.orientation, q_target);
  tf2::Quaternion q_next = q_curr.slerp(q_target, ratio);
  next.pose.orientation = tf2::toMsg(q_next);

  next.header.stamp = this->now();

  publisher_->publish(next);
  current_pose_ = next;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionSmootherNode>());
  rclcpp::shutdown();
  return 0;
}
