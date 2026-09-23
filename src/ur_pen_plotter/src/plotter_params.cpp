#include "ur_pen_plotter/plotter_params.hpp"

namespace ur_pen_plotter
{
namespace
{
double readDouble(rclcpp::Node & node, const std::string & name, double fallback)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<double>(name, fallback);
  }
  return node.get_parameter(name).as_double();
}

std::string readString(rclcpp::Node & node, const std::string & name, const std::string & fallback)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<std::string>(name, fallback);
  }
  return node.get_parameter(name).as_string();
}

bool readBool(rclcpp::Node & node, const std::string & name, bool fallback)
{
  if (!node.has_parameter(name)) {
    return node.declare_parameter<bool>(name, fallback);
  }
  return node.get_parameter(name).as_bool();
}
}  // namespace

PlotterParams loadPlotterParams(rclcpp::Node & node)
{
  PlotterParams params;

  params.plane_x = readDouble(node, "plane_x", params.plane_x);
  params.canvas_origin_z = readDouble(node, "canvas_origin_z", params.canvas_origin_z);
  params.pen_length = readDouble(node, "pen_length", params.pen_length);
  params.pen_lift = readDouble(node, "pen_lift", params.pen_lift);
  params.sample_step = readDouble(node, "sample_step", params.sample_step);

  params.circle_centre_u = readDouble(node, "circle_centre_u", params.circle_centre_u);
  params.circle_centre_v = readDouble(node, "circle_centre_v", params.circle_centre_v);
  params.circle_radius = readDouble(node, "circle_radius", params.circle_radius);

  params.vee_centre_u = readDouble(node, "vee_centre_u", params.vee_centre_u);
  params.vee_centre_v = readDouble(node, "vee_centre_v", params.vee_centre_v);
  params.vee_width = readDouble(node, "vee_width", params.vee_width);
  params.vee_height = readDouble(node, "vee_height", params.vee_height);

  params.a_centre_u = readDouble(node, "a_centre_u", params.a_centre_u);
  params.a_centre_v = readDouble(node, "a_centre_v", params.a_centre_v);
  params.a_width = readDouble(node, "a_width", params.a_width);
  params.a_height = readDouble(node, "a_height", params.a_height);
  params.a_bar_ratio = readDouble(node, "a_bar_ratio", params.a_bar_ratio);

  params.min_stroke_gap = readDouble(node, "min_stroke_gap", params.min_stroke_gap);
  params.max_tool_reach = readDouble(node, "max_tool_reach", params.max_tool_reach);
  params.shoulder_height = readDouble(node, "shoulder_height", params.shoulder_height);

  params.wait_for_button = readBool(node, "wait_for_button", params.wait_for_button);
  params.avoid_collisions = readBool(node, "avoid_collisions", params.avoid_collisions);
  params.cartesian_step = readDouble(node, "cartesian_step", params.cartesian_step);
  params.velocity_scaling = readDouble(node, "velocity_scaling", params.velocity_scaling);
  params.acceleration_scaling = readDouble(node, "acceleration_scaling", params.acceleration_scaling);
  params.planning_time = readDouble(node, "planning_time", params.planning_time);

  params.trail_line_width = readDouble(node, "trail_line_width", params.trail_line_width);
  params.trail_min_spacing = readDouble(node, "trail_min_spacing", params.trail_min_spacing);
  params.trail_csv = readString(node, "trail_csv", params.trail_csv);
  params.planned_csv = readString(node, "planned_csv", params.planned_csv);

  params.planning_group = readString(node, "planning_group", params.planning_group);
  return params;
}

Canvas makeCanvas(const PlotterParams & params)
{
  return Canvas(params.plane_x, params.canvas_origin_z, params.pen_length);
}

std::vector<Stroke> buildStrokes(const PlotterParams & params)
{
  std::vector<Stroke> strokes{
    makeCircle(
      "hinh_tron", Point2{params.circle_centre_u, params.circle_centre_v}, params.circle_radius,
      params.sample_step),
    makeLetterV(
      "chu_v", Point2{params.vee_centre_u, params.vee_centre_v}, params.vee_width,
      params.vee_height, params.sample_step),
  };

  const auto letter_a = makeLetterA(
    "chu_a", Point2{params.a_centre_u, params.a_centre_v}, params.a_width, params.a_height,
    params.a_bar_ratio, params.sample_step);
  strokes.insert(strokes.end(), letter_a.begin(), letter_a.end());
  return strokes;
}

}  // namespace ur_pen_plotter
