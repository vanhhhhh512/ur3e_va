cmake_minimum_required(VERSION 3.16)
project(ur_pen_plotter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# Phu thuoc cua thu vien hinh hoc (thuan tuy, khong dinh MoveIt) va cua node dieu khien.
set(GEOMETRY_DEPS rclcpp geometry_msgs)
set(MOTION_DEPS
  rclcpp_action
  visualization_msgs
  rviz_visual_tools
  control_msgs
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
)
foreach(dependency IN LISTS GEOMETRY_DEPS MOTION_DEPS)
  find_package(${dependency} REQUIRED)
endforeach()

# Thu vien hinh hoc thuan tuy: khong phu thuoc MoveIt nen test duoc ma khong can Gazebo.
add_library(pen_geometry
  src/canvas.cpp
  src/strokes.cpp
  src/layout.cpp
  src/plotter_params.cpp
)
target_include_directories(pen_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pen_geometry ${GEOMETRY_DEPS})

add_executable(draw_shapes_node src/draw_shapes_node.cpp src/ink_trail.cpp)
target_link_libraries(draw_shapes_node pen_geometry)
ament_target_dependencies(draw_shapes_node ${GEOMETRY_DEPS} ${MOTION_DEPS})

add_executable(stroke_preview src/stroke_preview.cpp)
target_link_libraries(stroke_preview pen_geometry)
ament_target_dependencies(stroke_preview ${GEOMETRY_DEPS})

install(TARGETS pen_geometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS draw_shapes_node stroke_preview
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# ur_moveit.launch.py doc gioi han khop va dong hoc tai
# <description_package>/config/<ur_type>/*.yaml, nen lay cac file do tu ur_description.
set(UR_TYPE "ur3e")
find_package(ur_description REQUIRED)
get_filename_component(UR_DESCRIPTION_SHARE "${ur_description_DIR}/.." ABSOLUTE)
if(NOT EXISTS "${UR_DESCRIPTION_SHARE}/config/${UR_TYPE}")
  message(FATAL_ERROR "Khong thay ${UR_DESCRIPTION_SHARE}/config/${UR_TYPE} trong ur_description")
endif()
install(DIRECTORY "${UR_DESCRIPTION_SHARE}/config/${UR_TYPE}"
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_strokes test/test_strokes.cpp)
  target_link_libraries(test_strokes pen_geometry)
  ament_target_dependencies(test_strokes ${GEOMETRY_DEPS})

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_yaml_safe test/test_urdf_yaml_safe.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(pen_geometry)
ament_package()
