# UR3e viết chữ trên mặt phẳng 2D — ROS 2 Humble + MoveIt 2 + Ignition Gazebo

## 1. Giới thiệu đề tài

Bài thực hành xây dựng một package ROS 2 điều khiển cánh tay **Universal Robots UR3e** cầm bút vẽ
lên một mặt phẳng dựng đứng trước mặt robot. Robot vẽ lần lượt ba hình:

1. một **đường tròn** khép kín,
2. **chữ V** — vẽ liền một nét,
3. **chữ A** — hai nét, giữa hai nét robot nhấc bút lên rồi hạ xuống chỗ mới.

V và A là hai chữ cái đầu trong tên người thực hiện (Việt Anh); đường tròn giữ lại theo yêu cầu
video demo của đề bài.

Trọng tâm của cách làm ở đây là **tách phần hình học ra khỏi phần điều khiển**. Hình vẽ được mô tả
bằng toạ độ 2D `(u, v)` ngay trên mặt phẳng giấy, không dính gì tới hệ toạ độ của robot; một lớp
riêng lo việc quy đổi sang pose 6 chiều cho MoveIt. Nhờ vậy muốn đổi chữ, đổi kích thước hay dời cả
bản vẽ sang chỗ khác thì chỉ sửa một file YAML, không đụng tới mã C++.

Trước khi robot cử động, chương trình tự kiểm tra bố trí: đo khoảng hở nhỏ nhất giữa các hình và
tầm với lớn nhất của cổ tay, vi phạm thì dừng ngay thay vì vẽ ra một bản hỏng. Trong lúc vẽ, vị trí
**thực tế** của đầu bút được lấy từ TF rồi dựng thành nét mực trong RViz2 và ghi ra CSV, nên sai
lệch giữa quỹ đạo dự định và quỹ đạo robot đi thật được đo bằng số chứ không phải nhìn bằng mắt.

* **Video demo:** *(điền link Google Drive)*
* **Mã nguồn:** https://github.com/vanhhhhh512/ur3e_va

---

## 2. Cấu trúc thư mục

```
ur3e_va/
├── .gitignore
├── README.md
├── scripts/
│   ├── sim_test.sh              # chạy trọn bộ không cần màn hình, tự chấm sai số
│   └── plot_trail.py            # vẽ lại bản vẽ từ CSV, so dự định với thực tế
└── src/ur_pen_plotter/          # package do sinh viên phát triển
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   ├── canvas.yaml          # toàn bộ hình học bản vẽ
    │   └── sim_controllers.yaml # controller cho Ignition, nới dung sai quỹ đạo
    ├── include/ur_pen_plotter/
    │   ├── canvas.hpp           # mặt phẳng vẽ: (u, v) ↔ pose của tool0
    │   ├── strokes.hpp          # sinh nét: đường tròn, chữ V, chữ A
    │   ├── layout.hpp           # kiểm tra khoảng hở và tầm với
    │   ├── plotter_params.hpp   # đọc tham số
    │   └── ink_trail.hpp        # lấy mẫu TF đầu bút → Marker + CSV
    ├── src/
    │   ├── canvas.cpp  strokes.cpp  layout.cpp  plotter_params.cpp  ink_trail.cpp
    │   ├── draw_shapes_node.cpp # node chính: lập kế hoạch và thực thi
    │   └── stroke_preview.cpp   # kiểm tra bố trí offline, không cần Gazebo
    ├── launch/
    │   ├── demo_all.launch.py   # một lệnh: Gazebo + MoveIt + RViz + node vẽ
    │   ├── bringup_sim.launch.py
    │   └── draw.launch.py
    ├── rviz/pen_plotter.rviz    # cấu hình sẵn Marker nét mực và waypoint
    ├── test/                    # test hình học + test URDF
    ├── urdf/ur3e_pen.urdf.xacro # UR3e gắn thêm cây bút vào tool0
    └── worlds/pen_board.sdf     # nền + tấm bảng trắng dựng tại x = 0.40 m
```

Ba launch file nằm trong `src/ur_pen_plotter/launch/`:

| Launch file | Nhiệm vụ |
| :--- | :--- |
| **`demo_all.launch.py`** | Chạy trọn bộ bằng một lệnh: Gazebo + ros2_control + MoveIt + RViz + node vẽ |
| **`bringup_sim.launch.py`** | Chỉ dựng môi trường mô phỏng: Gazebo (UR3e + bảng vẽ), controller, MoveIt, RViz |
| **`draw.launch.py`** | Chỉ chạy node điều khiển `draw_shapes_node`, dùng khi môi trường đã sẵn |

---

## 3. Yêu cầu hệ thống và cài đặt

| Thành phần | Phiên bản |
| :--- | :--- |
| Hệ điều hành | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill (Desktop) |
| Mô phỏng | Ignition Gazebo Fortress |
| Lập kế hoạch chuyển động | MoveIt 2 |

Cài ROS 2 Humble (bỏ qua nếu máy đã có):

```bash
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-humble-desktop
```

Các gói cho bài này:

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ur-simulation-gz \
  ros-humble-ur-moveit-config \
  ros-humble-ur-description \
  ros-humble-ign-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rviz-visual-tools \
  python3-colcon-common-extensions \
  python3-matplotlib
```

Gói tuỳ chọn, chỉ cần khi chạy `scripts/sim_test.sh` ở chế độ không màn hình:

```bash
sudo apt install -y xvfb ffmpeg imagemagick python3-pytest
```

> **Lưu ý về Python.** Nếu máy có Anaconda/Miniconda, `colcon` sẽ bắt nhầm Python của conda và báo
> `ModuleNotFoundError: No module named 'catkin_pkg'`. Chạy `conda deactivate` trước khi build,
> hoặc thêm `--cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3`.

---

## 4. Hướng dẫn build

```bash
git clone https://github.com/vanhhhhh512/ur3e_va.git
cd ur3e_va

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Cấu hình động học và giới hạn khớp của UR3e **không nằm trong repo**: `CMakeLists.txt` lấy thẳng
bốn file đó từ gói `ur_description` đã cài rồi cài kèm vào package lúc build, nên đổi phiên bản
`ur_description` thì thông số khớp cũng đi theo.

Về dung sai controller: trong Gazebo có trọng lực, giao diện điều khiển vị trí bám trễ vài độ. Để
`joint_trajectory_controller` không huỷ lệnh giữa chừng, package dùng file
`config/sim_controllers.yaml` riêng với dung sai nới lên 10 rad, truyền vào launch qua tham số
`runtime_config_package` — không phải sửa file trong `/opt/ros`.

---

## 5. Hướng dẫn chạy

### Cách 1 — một lệnh duy nhất: **`demo_all.launch.py`**

```bash
ros2 launch ur_pen_plotter demo_all.launch.py
```

**`demo_all.launch.py`** mở Gazebo, MoveIt, RViz rồi tự chạy node vẽ sau 25 giây. Máy yếu thì tăng thời gian chờ:
`demo_all.launch.py start_delay:=40.0`.

### Cách 2 — bấm nút rồi mới vẽ (dùng khi quay video): **`demo_all.launch.py`**

```bash
ros2 launch ur_pen_plotter demo_all.launch.py wait_for_button:=true
```

Robot đứng yên ở tư thế ban đầu, trên mặt phẳng vẽ đã hiện sẵn **chuỗi điểm trắng** là các waypoint
dự định (topic `/pen_waypoints`). Khi nào sẵn sàng quay thì bấm **Next** trong panel
`RvizVisualToolsGui` ở góc dưới bên trái RViz, robot mới bắt đầu. Nét mực thật vẽ đè lên chuỗi điểm
trắng đó nên nhìn được ngay robot bám quỹ đạo sát tới đâu.

### Cách 3 — hai terminal: **`bringup_sim.launch.py`** và **`draw.launch.py`**

```bash
# Terminal 1
ros2 launch ur_pen_plotter bringup_sim.launch.py

# Terminal 2
source install/setup.bash
ros2 launch ur_pen_plotter draw.launch.py trail_csv:=$PWD/out/trail.csv planned_csv:=$PWD/out/planned.csv
```

### Kiểm tra robot, joint state và controller

```bash
ros2 node list                    # /move_group, /robot_state_publisher, /controller_manager ...
ros2 topic hz /joint_states       # ~100 Hz
ros2 control list_controllers     # joint_state_broadcaster + joint_trajectory_controller: active
ros2 action list | grep follow_joint_trajectory
```

### Chạy tự động, không cần màn hình

```bash
./scripts/sim_test.sh
```

Script làm bốn bước: kiểm tra bố trí → bật Gazebo + MoveIt + RViz trên màn hình ảo Xvfb → vẽ và ghi
lại vệt bút → đối chiếu quỹ đạo rồi in sai số. Sản phẩm nằm trong `out/`: `ban_ve.png`, `rviz.png`,
`demo.mp4`, `planned.csv`, `trail.csv`.

---

## 6. Nguyên lý hoạt động

### 6.1 Hệ toạ độ mặt phẳng vẽ

Mặt phẳng vẽ vuông góc với trục X của world, cách đế robot đúng `plane_x`. Trên mặt phẳng dùng một
hệ 2D riêng: trục `u` chạy theo `-Y` của world (nhìn từ robot ra thì là chiều sang phải), trục `v`
chạy theo `+Z`. Mọi hình chỉ cần khai báo bằng `(u, v)`.

Lớp `Canvas` làm hai việc khi đổi `(u, v)` thành pose cho MoveIt:

* cố định hướng bút — trục Z của `tool0` hướng theo `+X` của world, tức bút chĩa vuông góc vào mặt
  phẳng, quaternion `w = y = √0.5`;
* lùi mặt bích lại đúng `pen_length`, vì thứ phải chạm mặt phẳng là **đầu bút** chứ không phải
  `tool0`. Cây bút là một link thật trong URDF, gắn cứng vào `tool0`.

### 6.2 Thoát điểm kỳ dị

Khi Gazebo vừa khởi động, UR3e nằm ở cấu hình toàn bộ khớp bằng 0, tức cánh tay duỗi thẳng — đúng
một điểm kỳ dị, ma trận Jacobian suy biến nên mọi lời gọi IK đều hỏng. Node vì thế luôn đưa robot
về tư thế gập trước khi làm bất cứ việc gì:

```
q_ready = [0, −π/2, π/2, −π/2, −π/2, 0]
```

### 6.3 Sinh tập waypoint

Mỗi hình gồm một hoặc nhiều `Stroke` — một nét liền mà bút không rời mặt phẳng. Điểm được lấy mẫu
cách nhau 2 mm dọc theo nét:

* **Đường tròn** — quét góc từ đỉnh trên theo chiều kim đồng hồ trọn 2π, điểm cuối trùng điểm đầu
  nên nét khép kín;
* **Chữ V** — hai đoạn thẳng nối trên-trái → đỉnh nhọn dưới → trên-phải, đi liền một nét;
* **Chữ A** — nét thứ nhất là khung nhọn dưới-trái → đỉnh → dưới-phải, nét thứ hai là thanh ngang.
  Ở độ cao `a_bar_ratio` tính từ đáy, hai cạnh xiên thu hẹp còn `(1 − a_bar_ratio)` lần nửa bề rộng,
  thanh ngang được cắt đúng bằng khoảng đó nên hai đầu chạm sát hai cạnh.

Hai nét của chữ A cắt nhau, nên mỗi `Stroke` mang thêm tên hình (`group`); phép kiểm tra khoảng hở
chỉ áp dụng giữa các hình khác nhau, không bắt lỗi hai nét trong cùng một chữ.

### 6.4 Kiểm tra bố trí trước khi chạy

`checkLayout()` duyệt mọi cặp nét khác hình, tính khoảng cách nhỏ nhất giữa hai tập điểm rồi so với
`min_stroke_gap`; đồng thời tính tầm với lớn nhất của `tool0` — đo từ **trục khớp vai** ở độ cao
0.152 m chứ không phải từ gốc toạ độ dưới sàn — rồi so với `max_tool_reach`. Sai một trong hai thì
chương trình dừng, chưa gửi lệnh nào xuống robot.

Bước này chạy riêng được, không cần bật Gazebo:

```bash
ros2 run ur_pen_plotter stroke_preview --ros-args \
  --params-file src/ur_pen_plotter/config/canvas.yaml -p planned_csv:=out/planned.csv
```

### 6.5 Trình tự thực thi một nét

```
đưa bút tới điểm chờ, cách mặt phẳng pen_lift
        ↓  hạ bút thẳng xuống mặt phẳng (computeCartesianPath)
    vẽ trọn nét (computeCartesianPath, bước 5 mm, avoid_collisions = true)
        ↓  nhấc bút
   sang nét kế tiếp bằng đường trượt ngang trong mặt phẳng nhấc bút
```

Nét đầu tiên phải xoay hướng tool rất nhiều so với tư thế `ready` nên lập kế hoạch trong không gian
khớp bằng OMPL; các nét sau chỉ trượt ngang nên đi thẳng bằng đường Cartesian, giữ nguyên nghiệm IK
đang dùng. Vẽ xong, robot xoay khớp vai 90° đỗ sang bên để không đứng che bản vẽ.

Hai chỗ phải xử lý vì MoveIt chọn nghiệm ngẫu nhiên:

* OMPL lấy mẫu ngẫu nhiên nên thỉnh thoảng trả về đường quét cẳng tay vào đế robot; MoveIt kiểm tra
  lại rồi từ chối. Mỗi lần lập kế hoạch vì thế được thử tối đa 5 lần.
* Tuỳ nghiệm IK mà MoveIt chọn cho điểm chờ, đường hạ bút thẳng đứng có thể cắt qua trạng thái tự
  va chạm và chỉ đi được một phần. Khi đó node quay về tư thế `ready` để MoveIt bốc nghiệm khác rồi
  tiếp cận lại, tối đa 3 lần; vẫn không được thì bỏ ràng buộc "hạ thẳng" và để OMPL tự tìm đường tới
  điểm chạm.

### 6.6 Vết mực và số liệu đo

Một timer 25 ms tra TF giữa `world` và `tool0`, cộng thêm `pen_length` dọc trục Z của tool để ra vị
trí **đầu bút thực tế**, rồi nối vào `visualization_msgs/Marker` kiểu `LINE_STRIP` — mỗi nét một
namespace và một màu, QoS *transient local* nên mở RViz muộn vẫn thấy đủ nét. Song song đó ghi CSV
gồm `stroke, t, x, y, z, u, v, plane_err`, trong đó `plane_err` là độ lệch của đầu bút so với mặt
phẳng vẽ.

`TransformListener` chạy trên luồng riêng, timer nằm trong nhóm callback riêng và node dùng
`MultiThreadedExecutor`. Chi tiết này quan trọng: nếu dùng chung executor với MoveIt thì lúc thi
hành quỹ đạo các callback bị nghẽn, TF tụt hậu hơn một giây và vệt bút ghi lại sẽ là vị trí của pha
di chuyển trước đó — bản vẽ méo hoàn toàn dù robot đi đúng.

---

## 7. Tham số bản vẽ

Toàn bộ hình học nằm trong `src/ur_pen_plotter/config/canvas.yaml`, sửa xong chạy lại là có hiệu
lực, không cần build lại.

| Tham số | Mặc định | Ý nghĩa |
| :--- | :---: | :--- |
| `plane_x` | `0.40` | Khoảng cách từ gốc robot tới mặt phẳng vẽ, tính tại đầu bút (m) |
| `canvas_origin_z` | `0.33` | Độ cao gốc canvas `(u=0, v=0)` (m) |
| `pen_length` | `0.10` | Chiều dài bút gắn trên `tool0`, phải khớp với URDF (m) |
| `pen_lift` | `0.05` | Khoảng nhấc bút khi chuyển giữa hai nét (m) |
| `sample_step` | `0.002` | Bước lấy mẫu dọc theo nét vẽ (m) |
| `circle_centre_u/v`, `circle_radius` | `−0.11 / 0 / 0.04` | Tâm và bán kính đường tròn |
| `vee_centre_u/v`, `vee_width`, `vee_height` | `0 / 0 / 0.08 / 0.11` | Tâm và kích thước chữ V |
| `a_centre_u/v`, `a_width`, `a_height` | `0.11 / 0 / 0.08 / 0.11` | Tâm và kích thước chữ A |
| `a_bar_ratio` | `0.4` | Độ cao thanh ngang chữ A tính từ đáy (0 = đáy, 1 = đỉnh) |
| `min_stroke_gap` | `0.03` | Khoảng hở tối thiểu giữa hai hình (m) |
| `max_tool_reach` | `0.45` | Chặn trên tầm với, đo từ trục khớp vai (m) |
| `wait_for_button` | `false` | `true` thì chờ bấm Next trong RViz rồi mới vẽ |
| `avoid_collisions` | `true` | Kiểm tra va chạm khi nội suy đường Cartesian |
| `cartesian_step` | `0.005` | Bước nội suy đường Cartesian (m) |
| `velocity_scaling` / `acceleration_scaling` | `0.15` | Hệ số tốc độ khi thi hành |

Dùng một file tham số khác mà không đụng file gốc:

```bash
ros2 launch ur_pen_plotter draw.launch.py params_file:=/duong/dan/canvas_cua_ban.yaml
```

---

## 8. Kiểm chứng và kết quả đo

Tám test hình học chạy được mà không cần Gazebo:

```bash
colcon test --packages-select ur_pen_plotter && colcon test-result --verbose
```

Nội dung kiểm tra: đường tròn khép kín và đúng bán kính, chữ V đúng ba đỉnh, chữ A đủ hai nét với
thanh ngang đúng độ cao và đúng bề rộng còn lại, khoảng hở giữa các hình đúng công thức, phép ánh xạ
canvas ↔ world là nghịch đảo của nhau, và chuỗi URDF sinh ra đọc được bằng YAML.

Kết quả một lần chạy đầy đủ với tham số mặc định:

| Chỉ số | Giá trị |
| :--- | :--- |
| Khoảng hở nhỏ nhất giữa các hình | **44.6 mm** (tròn ↔ V); các cặp còn lại 66–156 mm |
| Tầm với lớn nhất của `tool0` | 0.359 m (giới hạn 0.45 m) |
| Chiều dài nét | tròn 25.1 cm · V 23.4 cm · khung A 23.4 cm · ngang A 4.8 cm |
| Tỉ lệ đường Cartesian tính được | 100 % ở mọi đoạn |
| Sai số bám quỹ đạo, trung bình | tròn 0.50 mm · V 0.46 mm · khung A 0.52 mm · ngang A 0.48 mm |
| Sai số bám lớn nhất | 0.99 mm |
| Độ lệch đầu bút so với mặt phẳng vẽ | lớn nhất 0.07 mm |

---

## 9. Đối chiếu với yêu cầu bài thực hành

| Yêu cầu của đề | Đáp ứng ở đâu |
| :--- | :--- |
| Ubuntu 22.04 + ROS 2 Humble Desktop | Mục 3 |
| Chạy được UR3/UR3e simulation với Gazebo | **`bringup_sim.launch.py`** → `ur_simulation_gz` + Ignition Fortress |
| Kiểm tra robot, joint state, controller | Mục 5 |
| Package ROS 2 có launch file điều khiển UR3e viết chữ | Package `ur_pen_plotter`, launch file **`demo_all.launch.py`** |
| Viết chữ cái đầu trong tên sinh viên | Việt Anh → **V** và **A** |
| Chữ nằm trong mặt phẳng Cartesian tự chọn | Mặt phẳng `x = plane_x`, hệ canvas `(u, v)` — mục 6.1 |
| Tự thiết kế tập waypoint | `src/strokes.cpp`, lấy mẫu 2 mm — mục 6.3 |
| Chia chữ nhiều nét, nhấc đầu công tác giữa hai nét | Chữ A gồm 2 nét, `pen_lift = 5 cm` — mục 6.5 |
| Dùng MoveIt 2 để lập kế hoạch và thực thi | `MoveGroupInterface` + `computeCartesianPath` |
| Kích thước đủ lớn để quan sát rõ | Đường tròn Ø 8 cm, mỗi chữ 8 × 11 cm |
| Không vượt giới hạn khớp | Giới hạn khớp từ `ur_description`, thêm chặn `max_tool_reach` |
| Không self-collision | `avoid_collisions = true`, MoveIt kiểm tra lại mọi đường OMPL |
| Hiện đường đi end-effector trên RViz | Marker `/pen_ink` và waypoint `/pen_waypoints` — mục 6.6 |
| Video demo quỹ đạo hình tròn | `out/demo.mp4` do `scripts/sim_test.sh` sinh |
