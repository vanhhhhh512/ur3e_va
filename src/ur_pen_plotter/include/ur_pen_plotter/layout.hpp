#ifndef UR_PEN_PLOTTER__LAYOUT_HPP_
#define UR_PEN_PLOTTER__LAYOUT_HPP_

#include <string>
#include <vector>

#include <rclcpp/logger.hpp>

#include "ur_pen_plotter/canvas.hpp"
#include "ur_pen_plotter/plotter_params.hpp"
#include "ur_pen_plotter/strokes.hpp"

namespace ur_pen_plotter
{

/// Kiem tra truoc khi chay: moi net nam trong tam voi va cac net cach nhau du xa
/// de hinh tron khong ve chong len chu V.
bool checkLayout(
  const rclcpp::Logger & logger, const PlotterParams & params, const Canvas & canvas,
  const std::vector<Stroke> & strokes);

/// Ghi quy dao DU DINH ve (toa do canvas + toa do world) ra CSV de doi chieu voi vet thuc te.
bool writePlannedCsv(
  const std::string & path, const Canvas & canvas, const std::vector<Stroke> & strokes);

}  // namespace ur_pen_plotter

#endif  // UR_PEN_PLOTTER__LAYOUT_HPP_
