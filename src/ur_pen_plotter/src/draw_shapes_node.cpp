// Node chinh: UR3e ve HINH TRON, CHU V roi CHU A tren cung mot mat phang 2D.
//
// Moi hinh duoc mo ta bang mot hoac nhieu Stroke doc lap trong he toa do canvas (u, v);
// chu A gom hai net nen giua chung but duoc nhac len. Node chi lam ba viec:
//   1. dung danh sach Stroke tu tham so va kiem tra bo tri (cac hinh phai cach nhau),
//   2. voi moi Stroke: nhac but -> di chuyen -> ha but -> ve -> nhac but,
//   3. ghi lai vet but THUC TE (TF cua dau but) ra marker RViz va file CSV.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_visual_tools/remote_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "ur_pen_plotter/canvas.hpp"
#include "ur_pen_plotter/ink_trail.hpp"
#include "ur_pen_plotter/layout.hpp"
#include "ur_pen_plotter/plotter_params.hpp"
#include "ur_pen_plotter/strokes.hpp"

namespace ur_pen_plotter
{
namespace
{
using namespace std::chrono_literals;

/// Tu the gap goc de thoat cau hinh ky di (tat ca khop bang 0) luc Gazebo vua khoi dong.
const std::vector<double> kReadyJoints = {0.0, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, 0.0};

/// Tu the do sau khi ve xong - xoay khop vai 90 do de canh tay tranh sang mot ben,
/// khong dung chan truoc ban ve khi quan sat hoac quay phim.
const std::vector<double> kParkJoints = {M_PI_2, -M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, 0.0};

/// OMPL lay mau ngau nhien nen co luc tra ve duong khong hop le; thu lai vai lan.
constexpr int kPlanAttempts = 5;

/// So lan tiep can lai mot net khi duong ha but thang dung bi chan.
constexpr int kApproachAttempts = 3;

/// Hien cac waypoint du dinh trong RViz duoi dang chuoi diem tren mat phang ve.
void publishWaypoints(
  rclcpp::Node & node,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
  const std::string & frame, const Canvas & canvas, const std::vector<Stroke> & strokes,
  double spacing, double sample_step)
{
  const int stride = std::max(1, static_cast<int>(std::lround(spacing / sample_step)));
  int id = 0;
  for (const auto & stroke : strokes) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = node.now();
    marker.ns = "waypoint_" + stroke.name;
    marker.id = ++id;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.005;
    marker.color.r = 1.0F;
    marker.color.g = 1.0F;
    marker.color.b = 1.0F;
    marker.color.a = 0.6F;
    marker.lifetime = rclcpp::Duration(0, 0);

    for (size_t i = 0; i < stroke.points.size(); i += stride) {
      const auto pose = canvas.toolPoseAt(stroke.points[i], 0.0);
      geometry_msgs::msg::Point point;
      point.x = canvas.planeX();
      point.y = pose.position.y;
      point.z = pose.position.z;
      marker.points.push_back(point);
    }
    publisher->publish(marker);
  }
}
}  // namespace

class PenPlotter
{
public:
  PenPlotter(rclcpp::Node::SharedPtr node, PlotterParams params, std::shared_ptr<InkTrail> trail)
  : node_(std::move(node)), params_(std::move(params)), canvas_(makeCanvas(params_)),
    trail_(std::move(trail)),
    move_group_(node_, params_.planning_group)
  {
    move_group_.setPlanningTime(params_.planning_time);
    move_group_.setNumPlanningAttempts(10);
    move_group_.setMaxVelocityScalingFactor(params_.velocity_scaling);
    move_group_.setMaxAccelerationScalingFactor(params_.acceleration_scaling);
    trail_->configureFrames(move_group_.getPlanningFrame(), move_group_.getEndEffectorLink());
  }

  std::string planningFrame() {return move_group_.getPlanningFrame();}
  std::string toolLink() {return move_group_.getEndEffectorLink();}

  bool waitForController()
  {
    const auto client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
      node_, "/joint_trajectory_controller/follow_joint_trajectory");
    RCLCPP_INFO(node_->get_logger(), "Cho joint_trajectory_controller san sang...");
    if (!client->wait_for_action_server(90s)) {
      RCLCPP_ERROR(node_->get_logger(), "Controller khong san sang sau 90 giay.");
      return false;
    }
    RCLCPP_INFO(node_->get_logger(), "Controller da san sang, cho vat ly Gazebo on dinh 2 giay.");
    rclcpp::sleep_for(2s);
    return true;
  }

  /// Do tay sang mot ben sau khi ve xong de khong che ban ve.
  bool parkAside()
  {
    return moveToJoints(kParkJoints, "do tay sang ben");
  }

  bool escapeSingularity()
  {
    return moveToJoints(kReadyJoints, "thoat diem ky di");
  }

  bool drawStroke(const Stroke & stroke, bool first_stroke)
  {
    const auto hover_start = canvas_.toolPoseAt(stroke.points.front(), params_.pen_lift);
    const auto touch_start = canvas_.toolPoseAt(stroke.points.front(), 0.0);
    const auto hover_end = canvas_.toolPoseAt(stroke.points.back(), params_.pen_lift);

    const std::string label = "[" + stroke.name + "] ";
    if (!approachAndTouch(hover_start, touch_start, label, first_stroke)) {
      return false;
    }

    trail_->beginStroke(stroke.name, stroke.colour);
    trail_->penDown();
    rclcpp::sleep_for(150ms);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(stroke.points.size());
    for (const auto & point : stroke.points) {
      waypoints.push_back(canvas_.toolPoseAt(point, 0.0));
    }
    const bool drawn = cartesian(waypoints, (label + "VE").c_str());

    rclcpp::sleep_for(150ms);
    trail_->penUp();
    trail_->publish();

    if (!drawn) {
      return false;
    }
    if (!cartesian({hover_end}, (label + "nhac but len").c_str())) {
      RCLCPP_WARN(node_->get_logger(), "%sve xong nhung nhac but khong sach.", label.c_str());
    }
    return true;
  }

private:
  /// Dua but toi diem cho roi ha xuong mat phang. Duong ha but co the bi chan tuy nghiem
  /// IK ma MoveIt chon, khi do quay ve tu the 'ready' va tiep can lai.
  bool approachAndTouch(
    const geometry_msgs::msg::Pose & hover, const geometry_msgs::msg::Pose & touch,
    const std::string & label, bool first_stroke)
  {
    for (int attempt = 1; attempt <= kApproachAttempts; ++attempt) {
      if (attempt > 1 && !moveToJoints(kReadyJoints, "doi nghiem IK truoc khi tiep can lai")) {
        break;
      }
      // Net dau tien phai doi huong tool nhieu so voi tu the 'ready' nen lap ke hoach
      // trong khong gian khop; cac net sau chi truot ngang nen di thang Cartesian.
      const bool use_joint_planner = first_stroke || attempt > 1;
      const bool approached = use_joint_planner ?
        moveToPose(hover, (label + "toi diem cho phia tren net ve").c_str()) :
        transit(hover, (label + "di chuyen nhac but sang net moi").c_str());
      if (!approached) {
        return false;
      }
      if (cartesian({touch}, (label + "ha but xuong mat phang").c_str())) {
        return true;
      }
      RCLCPP_WARN(
        node_->get_logger(), "%sha but hong o lan %d/%d, se tiep can lai.", label.c_str(), attempt,
        kApproachAttempts);
    }
    // Khong nghiem IK nao cho ha but thang dung: bo rang buoc do, di thang toi diem cham.
    RCLCPP_WARN(
      node_->get_logger(), "%sha but thang dung khong duoc, lap ke hoach thang toi diem cham.",
      label.c_str());
    return moveToPose(touch, (label + "toi thang diem cham").c_str());
  }

  /// Lap ke hoach (co thu lai) cho muc tieu da dat truoc do, roi thi hanh.
  bool planAndExecute(const char * what)
  {
    for (int attempt = 1; attempt <= kPlanAttempts; ++attempt) {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_.clearPoseTargets();
        return move_group_.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      }
      RCLCPP_WARN(
        node_->get_logger(), "%s: lap ke hoach hong o lan %d/%d, thu lai.", what, attempt,
        kPlanAttempts);
    }
    move_group_.clearPoseTargets();
    RCLCPP_ERROR(node_->get_logger(), "%s: het %d lan thu van khong lap duoc ke hoach.", what,
      kPlanAttempts);
    return false;
  }

  bool moveToJoints(const std::vector<double> & joints, const char * what)
  {
    RCLCPP_INFO(node_->get_logger(), "Di chuyen khop de %s...", what);
    move_group_.setJointValueTarget(joints);
    return planAndExecute(what);
  }

  bool moveToPose(const geometry_msgs::msg::Pose & target, const char * what)
  {
    move_group_.setPoseTarget(target);
    return planAndExecute(what);
  }

  bool cartesian(const std::vector<geometry_msgs::msg::Pose> & waypoints, const char * what)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction =
      move_group_.computeCartesianPath(
      waypoints, params_.cartesian_step, 0.0, trajectory, params_.avoid_collisions);
    RCLCPP_INFO(node_->get_logger(), "%s: duong Cartesian dat %.1f%%.", what, fraction * 100.0);
    if (fraction <= 0.90) {
      RCLCPP_ERROR(node_->get_logger(), "%s: duong Cartesian khong du 90%%.", what);
      return false;
    }
    return move_group_.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS;
  }

  /// Di chuyen nhac but: uu tien duong thang Cartesian, khong duoc thi quay ve OMPL.
  bool transit(const geometry_msgs::msg::Pose & target, const char * what)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction =
      move_group_.computeCartesianPath(
      {target}, params_.cartesian_step, 0.0, trajectory, params_.avoid_collisions);
    if (fraction > 0.90 && move_group_.execute(trajectory) == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(node_->get_logger(), "%s: truot thang (%.1f%%).", what, fraction * 100.0);
      return true;
    }
    RCLCPP_WARN(node_->get_logger(), "%s: Cartesian chi %.1f%%, chuyen sang OMPL.", what, fraction * 100.0);
    return moveToPose(target, what);
  }

  rclcpp::Node::SharedPtr node_;
  PlotterParams params_;
  Canvas canvas_;
  std::shared_ptr<InkTrail> trail_;
  moveit::planning_interface::MoveGroupInterface move_group_;
};

}  // namespace ur_pen_plotter

int main(int argc, char ** argv)
{
  using namespace ur_pen_plotter;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto node = rclcpp::Node::make_shared("draw_shapes_node", options);
  const auto logger = node->get_logger();

  const PlotterParams params = loadPlotterParams(*node);
  const Canvas canvas = makeCanvas(params);
  const std::vector<Stroke> strokes = buildStrokes(params);

  if (!checkLayout(logger, params, canvas, strokes)) {
    rclcpp::shutdown();
    return 1;
  }
  if (!params.planned_csv.empty()) {
    writePlannedCsv(params.planned_csv, canvas, strokes);
    RCLCPP_INFO(logger, "Da ghi quy dao ke hoach: %s", params.planned_csv.c_str());
  }

  auto trail = std::make_shared<InkTrail>(
    node, canvas, params.trail_line_width, params.trail_min_spacing, params.trail_csv);

  // MultiThreadedExecutor de timer lay mau vet but khong bi callback cua MoveIt lam nghen.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  PenPlotter plotter(node, params, trail);

  const auto waypoint_publisher = node->create_publisher<visualization_msgs::msg::Marker>(
    "pen_waypoints", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  publishWaypoints(
    *node, waypoint_publisher, plotter.planningFrame(), canvas, strokes, 0.01,
    params.sample_step);
  RCLCPP_INFO(
    logger, "Mat phang ve x=%.3f m, but dai %.3f m, khung canvas goc z=%.3f m (%s -> %s).",
    params.plane_x, params.pen_length, params.canvas_origin_z, plotter.planningFrame().c_str(),
    plotter.toolLink().c_str());

  bool ok = plotter.waitForController();

  if (ok && params.wait_for_button) {
    rviz_visual_tools::RemoteControl remote(node);
    RCLCPP_INFO(logger, "Bam nut 'Next' trong panel RvizVisualToolsGui de bat dau ve.");
    remote.waitForNextStep("bat dau ve");
  }

  ok = ok && plotter.escapeSingularity();
  for (size_t i = 0; ok && i < strokes.size(); ++i) {
    ok = plotter.drawStroke(strokes[i], i == 0);
  }
  if (ok) {
    plotter.parkAside();
  }

  trail->publish();
  if (ok) {
    RCLCPP_INFO(
      logger, "XONG. Da lay %zu mau vet but, sai lech mat phang lon nhat %.2f mm.",
      trail->sampleCount(), trail->maxPlaneError() * 1000.0);
    RCLCPP_INFO(logger, "Net muc nam tren topic /pen_ink (QoS transient local).");
  } else {
    RCLCPP_ERROR(logger, "Chuoi ve bi dung giua chung.");
  }

  rclcpp::shutdown();
  spinner.join();
  return ok ? 0 : 1;
}
