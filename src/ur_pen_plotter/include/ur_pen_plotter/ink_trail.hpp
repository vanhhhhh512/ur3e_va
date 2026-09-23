#ifndef UR_PEN_PLOTTER__INK_TRAIL_HPP_
#define UR_PEN_PLOTTER__INK_TRAIL_HPP_

#include <array>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "ur_pen_plotter/canvas.hpp"

namespace ur_pen_plotter
{

/// Lay mau vi tri THUC TE cua dau but tu TF va ve lai thanh net muc trong RViz.
/// Moi net ve la mot marker rieng (namespace + mau rieng) nen hinh tron va chu V
/// khong bi noi lien voi nhau bang doan di chuyen nhac but.
class InkTrail
{
public:
  InkTrail(
    rclcpp::Node::SharedPtr node, const Canvas & canvas, double line_width, double min_spacing,
    const std::string & csv_path);
  ~InkTrail();

  void configureFrames(const std::string & planning_frame, const std::string & tool_link);

  void beginStroke(const std::string & name, const std::array<float, 3> & colour);
  void penDown();
  void penUp();
  void publish();

  size_t sampleCount() const;
  /// Sai lech lon nhat giua dau but va mat phang ve trong luc but ha (met).
  double maxPlaneError() const;

private:
  void sample();

  rclcpp::Node::SharedPtr node_;
  Canvas canvas_;
  double min_spacing_;
  std::string planning_frame_;
  std::string tool_link_;
  std::string stroke_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex mutex_;
  visualization_msgs::msg::Marker marker_;
  std::ofstream csv_;
  bool pen_down_{false};
  int marker_id_{0};
  size_t samples_{0};
  double max_plane_error_{0.0};
};

}  // namespace ur_pen_plotter

#endif  // UR_PEN_PLOTTER__INK_TRAIL_HPP_
