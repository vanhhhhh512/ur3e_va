#include "ur_pen_plotter/canvas.hpp"

#include <cmath>

namespace ur_pen_plotter
{

Canvas::Canvas(double plane_x, double origin_z, double pen_length)
: plane_x_(plane_x), origin_z_(origin_z), pen_length_(pen_length)
{
}

geometry_msgs::msg::Quaternion Canvas::penOrientation() const
{
  // Quay 90 do quanh truc Y: truc Z cua tool tu +Z world sang +X world.
  geometry_msgs::msg::Quaternion orientation;
  orientation.w = std::sqrt(0.5);
  orientation.x = 0.0;
  orientation.y = std::sqrt(0.5);
  orientation.z = 0.0;
  return orientation;
}

geometry_msgs::msg::Pose Canvas::toolPoseAt(const Point2 & uv, double lift) const
{
  geometry_msgs::msg::Pose pose;
  pose.orientation = penOrientation();
  pose.position.x = plane_x_ - pen_length_ - lift;
  pose.position.y = -uv.u;
  pose.position.z = origin_z_ + uv.v;
  return pose;
}

Point2 Canvas::toCanvas(double tool_y, double tool_z) const
{
  return Point2{-tool_y, tool_z - origin_z_};
}

double Canvas::liftOf(double tool_x) const
{
  return plane_x_ - pen_length_ - tool_x;
}

}  // namespace ur_pen_plotter
