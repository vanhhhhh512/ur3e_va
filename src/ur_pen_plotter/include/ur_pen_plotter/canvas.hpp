#ifndef UR_PEN_PLOTTER__CANVAS_HPP_
#define UR_PEN_PLOTTER__CANVAS_HPP_

#include <geometry_msgs/msg/pose.hpp>

namespace ur_pen_plotter
{

/// Mot diem tren mat phang ve, don vi met.
struct Point2
{
  double u{0.0};
  double v{0.0};
};

/// Mat phang ve dung thang truoc robot, vuong goc truc X cua world.
///
///   u  ->  -Y cua world (nhin tu goc robot ra truoc thi u chay sang phai)
///   v  ->  +Z cua world (chay len tren)
///
/// `pen_length` la chieu dai but gan vao tool0: robot lap ke hoach cho tool0,
/// con toa do (u, v) luon la vi tri DAU BUT cham mat phang.
class Canvas
{
public:
  Canvas(double plane_x, double origin_z, double pen_length);

  /// Huong but: truc Z cua tool huong theo +X world (but chia thang vao mat phang).
  geometry_msgs::msg::Quaternion penOrientation() const;

  /// Pose cua tool0 de dau but nam tai (u, v); lift > 0 la nhac but lui theo -X.
  geometry_msgs::msg::Pose toolPoseAt(const Point2 & uv, double lift = 0.0) const;

  /// Nghich dao: tu vi tri tool0 trong world suy ra toa do dau but tren canvas.
  Point2 toCanvas(double tool_y, double tool_z) const;

  /// Khoang cach tu dau but toi mat phang (0 = dang cham, > 0 = dang nhac).
  double liftOf(double tool_x) const;

  double planeX() const {return plane_x_;}
  double originZ() const {return origin_z_;}
  double penLength() const {return pen_length_;}

private:
  double plane_x_;
  double origin_z_;
  double pen_length_;
};

}  // namespace ur_pen_plotter

#endif  // UR_PEN_PLOTTER__CANVAS_HPP_
