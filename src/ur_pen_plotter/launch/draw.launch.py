# Chay node ve: hinh tron truoc, chu V sau. Moi truong mo phong phai chay san
# (launch bringup_sim.launch.py o terminal khac).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("ur_pen_plotter")
    params_file = LaunchConfiguration("params_file")
    trail_csv = LaunchConfiguration("trail_csv")
    planned_csv = LaunchConfiguration("planned_csv")

    draw_node = Node(
        package="ur_pen_plotter",
        executable="draw_shapes_node",
        name="draw_shapes_node",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "trail_csv": trail_csv,
                "planned_csv": planned_csv,
                "wait_for_button": ParameterValue(
                    LaunchConfiguration("wait_for_button"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution([package, "config", "canvas.yaml"]),
                description="File tham so hinh hoc ban ve.",
            ),
            DeclareLaunchArgument(
                "wait_for_button",
                default_value="false",
                description="Cho bam nut Next trong panel RvizVisualToolsGui roi moi ve.",
            ),
            DeclareLaunchArgument(
                "trail_csv", default_value="", description="Duong dan CSV ghi vet but THUC TE."
            ),
            DeclareLaunchArgument(
                "planned_csv", default_value="", description="Duong dan CSV ghi quy dao DU DINH."
            ),
            draw_node,
        ]
    )
