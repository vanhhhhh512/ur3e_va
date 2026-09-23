#include "ur_pen_plotter/layout.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>

#include <rclcpp/logging.hpp>

namespace ur_pen_plotter
{
namespace
{
/// Tam voi cua UR3e tinh tu truc khop vai, khong phai tu goc toa do tren san.
double toolReach(const geometry_msgs::msg::Pose & pose, double shoulder_height)
{
  const double dz = pose.position.z - shoulder_height;
  return std::sqrt(
    pose.position.x * pose.position.x + pose.position.y * pose.position.y + dz * dz);
}
}  // namespace

bool checkLayout(
  const rclcpp::Logger & logger, const PlotterParams & params, const Canvas & canvas,
  const std::vector<Stroke> & strokes)
{
  bool ok = true;

  for (const auto & stroke : strokes) {
    const auto bounds = boundsOf(stroke);
    double reach = 0.0;
    for (const auto & point : stroke.points) {
      reach = std::max(
        reach, toolReach(canvas.toolPoseAt(point, params.pen_lift), params.shoulder_height));
    }
    RCLCPP_INFO(
      logger, "Net '%s': %zu diem, dai %.1f cm, u=[%+.3f, %+.3f] v=[%+.3f, %+.3f], tam voi %.3f m.",
      stroke.name.c_str(), stroke.points.size(), pathLength(stroke) * 100.0, bounds.u_min,
      bounds.u_max, bounds.v_min, bounds.v_max, reach);
    if (reach > params.max_tool_reach) {
      RCLCPP_ERROR(
        logger, "Net '%s' vuot tam voi cho phep: %.3f > %.3f m.", stroke.name.c_str(), reach,
        params.max_tool_reach);
      ok = false;
    }
  }

  for (size_t i = 0; i + 1 < strokes.size(); ++i) {
    for (size_t j = i + 1; j < strokes.size(); ++j) {
      // Hai net cua cung mot chu duoc phep cat nhau (thanh ngang chu A).
      if (strokes[i].group == strokes[j].group) {
        continue;
      }
      const double gap = minimumGap(strokes[i], strokes[j]);
      RCLCPP_INFO(
        logger, "Khoang ho '%s' <-> '%s' = %.1f mm (yeu cau >= %.1f mm).", strokes[i].name.c_str(),
        strokes[j].name.c_str(), gap * 1000.0, params.min_stroke_gap * 1000.0);
      if (gap < params.min_stroke_gap) {
        RCLCPP_ERROR(logger, "Hai hinh qua gan nhau nen se ve de len nhau.");
        ok = false;
      }
    }
  }
  return ok;
}

bool writePlannedCsv(
  const std::string & path, const Canvas & canvas, const std::vector<Stroke> & strokes)
{
  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }
  file << "stroke,u,v,x,y,z\n";
  for (const auto & stroke : strokes) {
    for (const auto & point : stroke.points) {
      const auto pose = canvas.toolPoseAt(point, 0.0);
      file << stroke.name << ',' << point.u << ',' << point.v << ',' << canvas.planeX() << ','
           << pose.position.y << ',' << pose.position.z << '\n';
    }
  }
  return true;
}

}  // namespace ur_pen_plotter
