// Kiem tra hinh hoc ban ve ma khong can Gazebo: hinh dang dung, hai hinh khong dinh nhau,
// va phep anh xa canvas <-> world la nghich dao cua nhau.
#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "ur_pen_plotter/canvas.hpp"
#include "ur_pen_plotter/strokes.hpp"

using ur_pen_plotter::Canvas;
using ur_pen_plotter::Point2;

TEST(Strokes, DuongTronKinVaDungBanKinh)
{
  const Point2 centre{-0.09, 0.0};
  const double radius = 0.045;
  const auto circle = ur_pen_plotter::makeCircle("tron", centre, radius, 0.002);

  ASSERT_GE(circle.points.size(), 24u);
  for (const auto & point : circle.points) {
    EXPECT_NEAR(std::hypot(point.u - centre.u, point.v - centre.v), radius, 1e-9);
  }
  // Diem dau trung diem cuoi -> net khep kin.
  EXPECT_NEAR(circle.points.front().u, circle.points.back().u, 1e-9);
  EXPECT_NEAR(circle.points.front().v, circle.points.back().v, 1e-9);
  EXPECT_NEAR(ur_pen_plotter::pathLength(circle), 2.0 * M_PI * radius, 1e-3);
}

TEST(Strokes, ChuVDungBaDinh)
{
  const Point2 centre{0.09, 0.0};
  const double width = 0.09;
  const double height = 0.12;
  const auto vee = ur_pen_plotter::makeLetterV("v", centre, width, height, 0.002);

  EXPECT_NEAR(vee.points.front().u, centre.u - width / 2.0, 1e-9);
  EXPECT_NEAR(vee.points.front().v, centre.v + height / 2.0, 1e-9);
  EXPECT_NEAR(vee.points.back().u, centre.u + width / 2.0, 1e-9);
  EXPECT_NEAR(vee.points.back().v, centre.v + height / 2.0, 1e-9);

  // Dinh nhon duoi cung la diem thap nhat va nam dung giua.
  auto lowest = vee.points.front();
  for (const auto & point : vee.points) {
    if (point.v < lowest.v) {
      lowest = point;
    }
  }
  EXPECT_NEAR(lowest.u, centre.u, 1e-9);
  EXPECT_NEAR(lowest.v, centre.v - height / 2.0, 1e-9);

  const auto bounds = ur_pen_plotter::boundsOf(vee);
  EXPECT_NEAR(bounds.u_max - bounds.u_min, width, 1e-9);
  EXPECT_NEAR(bounds.v_max - bounds.v_min, height, 1e-9);
}

TEST(Strokes, ChuAGomHaiNetVaThanhNgangDungCho)
{
  const Point2 centre{0.12, 0.0};
  const double width = 0.08;
  const double height = 0.11;
  const double bar_ratio = 0.4;
  const auto parts = ur_pen_plotter::makeLetterA("chu_a", centre, width, height, bar_ratio, 0.002);

  ASSERT_EQ(parts.size(), 2u);
  // Hai net phai cung mot group, neu khong kiem tra khoang ho se bao chu A tu ve chong.
  EXPECT_EQ(parts[0].group, parts[1].group);

  const auto & frame = parts[0];
  EXPECT_NEAR(frame.points.front().u, centre.u - width / 2.0, 1e-9);
  EXPECT_NEAR(frame.points.front().v, centre.v - height / 2.0, 1e-9);
  EXPECT_NEAR(frame.points.back().u, centre.u + width / 2.0, 1e-9);
  EXPECT_NEAR(frame.points.back().v, centre.v - height / 2.0, 1e-9);

  auto highest = frame.points.front();
  for (const auto & point : frame.points) {
    if (point.v > highest.v) {
      highest = point;
    }
  }
  EXPECT_NEAR(highest.u, centre.u, 1e-9);
  EXPECT_NEAR(highest.v, centre.v + height / 2.0, 1e-9);

  // Thanh ngang phai nam ngang, dung do cao va dung be rong con lai cua hai canh.
  const auto & bar = parts[1];
  const double expected_v = centre.v - height / 2.0 + bar_ratio * height;
  const double expected_half = (width / 2.0) * (1.0 - bar_ratio);
  for (const auto & point : bar.points) {
    EXPECT_NEAR(point.v, expected_v, 1e-9);
  }
  EXPECT_NEAR(bar.points.front().u, centre.u - expected_half, 1e-9);
  EXPECT_NEAR(bar.points.back().u, centre.u + expected_half, 1e-9);
}

TEST(Strokes, BaHinhCachNhauDuXa)
{
  // Dung dung bo tri mac dinh trong config/canvas.yaml.
  const Point2 circle_centre{-0.12, 0.0};
  const double radius = 0.04;
  const auto circle = ur_pen_plotter::makeCircle("tron", circle_centre, radius, 0.002);
  const auto vee = ur_pen_plotter::makeLetterV("v", Point2{0.0, 0.0}, 0.08, 0.11, 0.002);
  const auto letter_a =
    ur_pen_plotter::makeLetterA("chu_a", Point2{0.12, 0.0}, 0.08, 0.11, 0.4, 0.002);

  // Voi mot duong tron, khoang ho toi net khac luon bang khoang cach ngan nhat tu TAM
  // toi net do tru di ban kinh (diem gan nhat nam tren canh chu V, khong phai o goc).
  double nearest_to_centre = std::numeric_limits<double>::max();
  for (const auto & point : vee.points) {
    nearest_to_centre = std::min(
      nearest_to_centre, std::hypot(point.u - circle_centre.u, point.v - circle_centre.v));
  }
  EXPECT_NEAR(ur_pen_plotter::minimumGap(circle, vee), nearest_to_centre - radius, 1e-3);

  const double gap_min = 0.03;
  EXPECT_GT(ur_pen_plotter::minimumGap(circle, vee), gap_min);
  EXPECT_GT(ur_pen_plotter::minimumGap(vee, letter_a[0]), gap_min);
  EXPECT_GT(ur_pen_plotter::minimumGap(vee, letter_a[1]), gap_min);
  EXPECT_GT(ur_pen_plotter::minimumGap(circle, letter_a[0]), gap_min);
}

TEST(Canvas, AnhXaCanvasVaWorldLaNghichDao)
{
  const Canvas canvas(0.40, 0.28, 0.10);
  const Point2 uv{0.07, -0.05};

  const auto pose = canvas.toolPoseAt(uv, 0.0);
  // tool0 lui lai dung bang chieu dai but so voi mat phang.
  EXPECT_NEAR(pose.position.x, 0.30, 1e-12);
  EXPECT_NEAR(canvas.liftOf(pose.position.x), 0.0, 1e-12);

  const auto back = canvas.toCanvas(pose.position.y, pose.position.z);
  EXPECT_NEAR(back.u, uv.u, 1e-12);
  EXPECT_NEAR(back.v, uv.v, 1e-12);

  const auto lifted = canvas.toolPoseAt(uv, 0.05);
  EXPECT_NEAR(canvas.liftOf(lifted.position.x), 0.05, 1e-12);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
