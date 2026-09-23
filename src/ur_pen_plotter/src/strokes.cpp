#include "ur_pen_plotter/strokes.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ur_pen_plotter
{
namespace
{
constexpr double kMinStep = 1.0e-4;

void appendSegment(
  std::vector<Point2> & points, const Point2 & from, const Point2 & to, double step,
  bool skip_first)
{
  const double length = std::hypot(to.u - from.u, to.v - from.v);
  const int count = std::max(1, static_cast<int>(std::ceil(length / std::max(step, kMinStep))));
  for (int i = skip_first ? 1 : 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    points.push_back(Point2{from.u + (to.u - from.u) * t, from.v + (to.v - from.v) * t});
  }
}
}  // namespace

Stroke makeCircle(const std::string & name, Point2 centre, double radius, double sample_step)
{
  Stroke stroke;
  stroke.name = name;
  stroke.group = name;
  stroke.colour = {{0.10F, 0.85F, 1.00F}};

  const double circumference = 2.0 * M_PI * radius;
  const int segments =
    std::max(24, static_cast<int>(std::ceil(circumference / std::max(sample_step, kMinStep))));
  stroke.points.reserve(segments + 1);
  for (int i = 0; i <= segments; ++i) {
    // Bat dau tu dinh tren (goc +90 do) roi giam goc -> chay theo chieu kim dong ho.
    const double angle = M_PI_2 - 2.0 * M_PI * static_cast<double>(i) / segments;
    stroke.points.push_back(
      Point2{centre.u + radius * std::cos(angle), centre.v + radius * std::sin(angle)});
  }
  return stroke;
}

Stroke makeLetterV(
  const std::string & name, Point2 centre, double width, double height, double sample_step)
{
  Stroke stroke;
  stroke.name = name;
  stroke.group = name;
  stroke.colour = {{1.00F, 0.35F, 0.05F}};

  const double half_width = width / 2.0;
  const double half_height = height / 2.0;
  const Point2 top_left{centre.u - half_width, centre.v + half_height};
  const Point2 apex{centre.u, centre.v - half_height};
  const Point2 top_right{centre.u + half_width, centre.v + half_height};

  appendSegment(stroke.points, top_left, apex, sample_step, false);
  appendSegment(stroke.points, apex, top_right, sample_step, true);
  return stroke;
}

std::vector<Stroke> makeLetterA(
  const std::string & group, Point2 centre, double width, double height, double bar_ratio,
  double sample_step)
{
  const double half_width = width / 2.0;
  const double half_height = height / 2.0;
  const Point2 bottom_left{centre.u - half_width, centre.v - half_height};
  const Point2 apex{centre.u, centre.v + half_height};
  const Point2 bottom_right{centre.u + half_width, centre.v - half_height};

  Stroke frame;
  frame.name = group + "_khung";
  frame.group = group;
  frame.colour = {{0.45F, 0.95F, 0.30F}};
  appendSegment(frame.points, bottom_left, apex, sample_step, false);
  appendSegment(frame.points, apex, bottom_right, sample_step, true);

  // Hai canh chu A thu hep tuyen tinh tu day len dinh, nen o do cao bar_ratio (tinh tu
  // day) nua be rong con lai la half_width * (1 - bar_ratio).
  const double bar_v = centre.v - half_height + bar_ratio * height;
  const double bar_half_width = half_width * (1.0 - bar_ratio);

  Stroke bar;
  bar.name = group + "_ngang";
  bar.group = group;
  bar.colour = {{0.45F, 0.95F, 0.30F}};
  appendSegment(
    bar.points, Point2{centre.u - bar_half_width, bar_v},
    Point2{centre.u + bar_half_width, bar_v}, sample_step, false);

  return {frame, bar};
}

Bounds boundsOf(const Stroke & stroke)
{
  Bounds bounds{
    std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
    std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
  for (const auto & point : stroke.points) {
    bounds.u_min = std::min(bounds.u_min, point.u);
    bounds.u_max = std::max(bounds.u_max, point.u);
    bounds.v_min = std::min(bounds.v_min, point.v);
    bounds.v_max = std::max(bounds.v_max, point.v);
  }
  return bounds;
}

double minimumGap(const Stroke & first, const Stroke & second)
{
  double gap = std::numeric_limits<double>::max();
  for (const auto & a : first.points) {
    for (const auto & b : second.points) {
      gap = std::min(gap, std::hypot(a.u - b.u, a.v - b.v));
    }
  }
  return gap;
}

double pathLength(const Stroke & stroke)
{
  double length = 0.0;
  for (size_t i = 1; i < stroke.points.size(); ++i) {
    length += std::hypot(
      stroke.points[i].u - stroke.points[i - 1].u, stroke.points[i].v - stroke.points[i - 1].v);
  }
  return length;
}

}  // namespace ur_pen_plotter
