#include "ur_pen_plotter/ink_trail.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>

namespace ur_pen_plotter
{
using namespace std::chrono_literals;

InkTrail::InkTrail(
  rclcpp::Node::SharedPtr node, const Canvas & canvas, double line_width, double min_spacing,
  const std::string & csv_path)
: node_(std::move(node)), canvas_(canvas), min_spacing_(min_spacing)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  // spin_thread = true: TransformListener co luong rieng de doc /tf, khong phu thuoc
  // executor dung chung voi MoveIt. Dung chung thi TF tut hau va vet but lay mau sai cho.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local();
  publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("pen_ink", qos);

  marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_.action = visualization_msgs::msg::Marker::ADD;
  marker_.pose.orientation.w = 1.0;
  marker_.scale.x = std::max(0.001, line_width);
  marker_.color.a = 1.0F;
  marker_.frame_locked = true;
  marker_.lifetime = rclcpp::Duration(0, 0);

  if (!csv_path.empty()) {
    csv_.open(csv_path);
    if (csv_.is_open()) {
      csv_ << "stroke,t,x,y,z,u,v,plane_err\n";
    } else {
      RCLCPP_WARN(node_->get_logger(), "Khong mo duoc file CSV: %s", csv_path.c_str());
    }
  }

  // Nhom callback rieng cho timer lay mau, khong xep hang sau cac callback cua MoveIt.
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = node_->create_wall_timer(25ms, [this]() {sample();}, callback_group_);
}

InkTrail::~InkTrail()
{
  if (csv_.is_open()) {
    csv_.close();
  }
}

void InkTrail::configureFrames(const std::string & planning_frame, const std::string & tool_link)
{
  std::lock_guard<std::mutex> lock(mutex_);
  planning_frame_ = planning_frame;
  tool_link_ = tool_link;
  marker_.header.frame_id = planning_frame;
}

void InkTrail::beginStroke(const std::string & name, const std::array<float, 3> & colour)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!marker_.points.empty()) {
    marker_.header.stamp = node_->now();
    publisher_->publish(marker_);
  }
  ++marker_id_;
  stroke_name_ = name;
  marker_.id = marker_id_;
  marker_.ns = "net_" + name;
  marker_.points.clear();
  marker_.color.r = colour[0];
  marker_.color.g = colour[1];
  marker_.color.b = colour[2];
}

void InkTrail::penDown()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pen_down_ = true;
}

void InkTrail::penUp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  pen_down_ = false;
}

void InkTrail::publish()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (marker_.points.empty()) {
    return;
  }
  marker_.header.stamp = node_->now();
  publisher_->publish(marker_);
}

size_t InkTrail::sampleCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return samples_;
}

double InkTrail::maxPlaneError() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return max_plane_error_;
}

void InkTrail::sample()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!pen_down_ || planning_frame_.empty() || tool_link_.empty()) {
    return;
  }

  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(planning_frame_, tool_link_, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "Chua doc duoc TF cua but: %s",
      exception.what());
    return;
  }

  // Dau but nam cach tool0 mot doan pen_length doc theo truc Z cua tool.
  const tf2::Quaternion rotation(
    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z,
    tf.transform.rotation.w);
  const tf2::Vector3 tip_offset = tf2::quatRotate(rotation, tf2::Vector3(0.0, 0.0, canvas_.penLength()));

  geometry_msgs::msg::Point tip;
  tip.x = tf.transform.translation.x + tip_offset.x();
  tip.y = tf.transform.translation.y + tip_offset.y();
  tip.z = tf.transform.translation.z + tip_offset.z();

  if (!marker_.points.empty()) {
    const auto & last = marker_.points.back();
    const double moved =
      std::sqrt(
      std::pow(tip.x - last.x, 2) + std::pow(tip.y - last.y, 2) + std::pow(tip.z - last.z, 2));
    if (moved < min_spacing_) {
      return;
    }
  }

  marker_.header.frame_id = planning_frame_;
  marker_.header.stamp = node_->now();
  marker_.points.push_back(tip);
  publisher_->publish(marker_);

  ++samples_;
  const double plane_error = std::abs(canvas_.planeX() - tip.x);
  max_plane_error_ = std::max(max_plane_error_, plane_error);

  if (csv_.is_open()) {
    const Point2 uv = canvas_.toCanvas(tip.y, tip.z);
    csv_ << stroke_name_ << ',' << node_->now().seconds() << ',' << tip.x << ',' << tip.y << ','
         << tip.z << ',' << uv.u << ',' << uv.v << ',' << plane_error << '\n';
  }
}

}  // namespace ur_pen_plotter
