#ifndef UR_PEN_PLOTTER__STROKES_HPP_
#define UR_PEN_PLOTTER__STROKES_HPP_

#include <array>
#include <string>
#include <vector>

#include "ur_pen_plotter/canvas.hpp"

namespace ur_pen_plotter
{

/// Mot net ve lien tuc: but ha xuong o diem dau, nhac len o diem cuoi.
/// `group` la ten hinh chua net do; cac net cung mot group duoc phep cat nhau.
struct Stroke
{
  std::string name;
  std::string group;
  std::array<float, 3> colour{{0.0F, 1.0F, 0.0F}};
  std::vector<Point2> points;
};

struct Bounds
{
  double u_min{0.0};
  double u_max{0.0};
  double v_min{0.0};
  double v_max{0.0};
};

/// Duong tron kin, bat dau tu dinh tren va chay theo chieu kim dong ho.
Stroke makeCircle(const std::string & name, Point2 centre, double radius, double sample_step);

/// Chu V: tren-trai -> dinh nhon duoi -> tren-phai (mot net lien).
Stroke makeLetterV(
  const std::string & name, Point2 centre, double width, double height, double sample_step);

/// Chu A: HAI net, giua hai net but duoc nhac len.
///   net 1 - duoi-trai -> dinh nhon tren -> duoi-phai
///   net 2 - thanh ngang, dat o do cao bar_ratio tinh tu day chu
std::vector<Stroke> makeLetterA(
  const std::string & group, Point2 centre, double width, double height, double bar_ratio,
  double sample_step);

Bounds boundsOf(const Stroke & stroke);

/// Khoang cach nho nhat giua hai net - dung de bao dam hai hinh khong ve de len nhau.
double minimumGap(const Stroke & first, const Stroke & second);

double pathLength(const Stroke & stroke);

}  // namespace ur_pen_plotter

#endif  // UR_PEN_PLOTTER__STROKES_HPP_
