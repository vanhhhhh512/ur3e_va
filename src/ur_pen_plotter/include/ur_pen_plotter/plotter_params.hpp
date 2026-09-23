#ifndef UR_PEN_PLOTTER__PLOTTER_PARAMS_HPP_
#define UR_PEN_PLOTTER__PLOTTER_PARAMS_HPP_

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ur_pen_plotter/canvas.hpp"
#include "ur_pen_plotter/strokes.hpp"

namespace ur_pen_plotter
{

struct PlotterParams
{
  // Mat phang ve
  double plane_x{0.40};
  double canvas_origin_z{0.33};
  double pen_length{0.10};
  double pen_lift{0.05};
  double sample_step{0.002};

  // Hinh tron
  double circle_centre_u{-0.11};
  double circle_centre_v{0.0};
  double circle_radius{0.04};

  // Chu V
  double vee_centre_u{0.0};
  double vee_centre_v{0.0};
  double vee_width{0.08};
  double vee_height{0.11};

  // Chu A
  double a_centre_u{0.11};
  double a_centre_v{0.0};
  double a_width{0.08};
  double a_height{0.11};
  double a_bar_ratio{0.4};

  // Rang buoc bo tri
  double min_stroke_gap{0.03};
  double max_tool_reach{0.45};   // UR3e voi toi 0.50 m tinh tu truc vai
  double shoulder_height{0.1519};

  // Thuc thi
  bool wait_for_button{false};   // cho bam nut Next trong RViz roi moi ve
  bool avoid_collisions{true};
  double cartesian_step{0.005};
  double velocity_scaling{0.15};
  double acceleration_scaling{0.15};
  double planning_time{10.0};

  // Hien thi + ghi log
  double trail_line_width{0.006};
  double trail_min_spacing{0.0015};
  std::string trail_csv{""};
  std::string planned_csv{""};

  std::string planning_group{"ur_manipulator"};
};

/// Doc tham so theo kieu "khai bao neu chua co" - MoveIt da tu khai bao mot so ten khac.
PlotterParams loadPlotterParams(rclcpp::Node & node);

Canvas makeCanvas(const PlotterParams & params);

/// Thu tu ve: hinh tron truoc, chu V sau.
std::vector<Stroke> buildStrokes(const PlotterParams & params);

}  // namespace ur_pen_plotter

#endif  // UR_PEN_PLOTTER__PLOTTER_PARAMS_HPP_
