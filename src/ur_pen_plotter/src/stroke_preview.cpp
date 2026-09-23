// Cong cu kiem tra bo tri KHONG can Gazebo/MoveIt: dung danh sach net tu dung tham so,
// kiem tra khoang ho + tam voi, roi ghi quy dao du dinh ra CSV de ve lai bang matplotlib.
//
//   ros2 run ur_pen_plotter stroke_preview --ros-args
//        --params-file <canvas.yaml> -p planned_csv:=/ws/out/planned.csv
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "ur_pen_plotter/layout.hpp"
#include "ur_pen_plotter/plotter_params.hpp"

int main(int argc, char ** argv)
{
  using namespace ur_pen_plotter;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  const auto node = rclcpp::Node::make_shared("stroke_preview", options);
  const auto logger = node->get_logger();

  const PlotterParams params = loadPlotterParams(*node);
  const Canvas canvas = makeCanvas(params);
  const std::vector<Stroke> strokes = buildStrokes(params);

  const bool layout_ok = checkLayout(logger, params, canvas, strokes);

  if (!params.planned_csv.empty()) {
    if (writePlannedCsv(params.planned_csv, canvas, strokes)) {
      RCLCPP_INFO(logger, "Da ghi quy dao du dinh: %s", params.planned_csv.c_str());
    } else {
      RCLCPP_ERROR(logger, "Khong ghi duoc file %s", params.planned_csv.c_str());
      rclcpp::shutdown();
      return 2;
    }
  }

  RCLCPP_INFO(logger, layout_ok ? "Bo tri HOP LE." : "Bo tri KHONG hop le.");
  rclcpp::shutdown();
  return layout_ok ? 0 : 1;
}
