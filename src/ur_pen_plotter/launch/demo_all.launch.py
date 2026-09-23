# Mot lenh chay tron bo: Gazebo (UR3e) -> ros2_control -> MoveIt -> RViz -> node ve.
#
#   ros2 launch ur_pen_plotter demo_all.launch.py
#
# Node ve duoc hoan lai start_delay giay vi MoveGroupInterface can move_group da san sang
# truoc khi khoi tao. Muon chu dong tung buoc thi dung hai launch rieng (bringup_sim.launch.py
# o mot terminal, draw.launch.py o terminal khac).
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare("ur_pen_plotter")

    bringup = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource([package, "/launch", "/bringup_sim.launch.py"]),
        launch_arguments={
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )])

    draw = TimerAction(
        period=LaunchConfiguration("start_delay"),
        actions=[GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource([package, "/launch", "/draw.launch.py"]),
            launch_arguments={
                "trail_csv": LaunchConfiguration("trail_csv"),
                "planned_csv": LaunchConfiguration("planned_csv"),
                "wait_for_button": LaunchConfiguration("wait_for_button"),
            }.items(),
        )])],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gazebo_gui", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument(
                "start_delay",
                default_value="25.0",
                description="So giay cho Gazebo va MoveIt san sang truoc khi chay node ve.",
            ),
            DeclareLaunchArgument("wait_for_button", default_value="false"),
            DeclareLaunchArgument("trail_csv", default_value=""),
            DeclareLaunchArgument("planned_csv", default_value=""),
            bringup,
            draw,
        ]
    )
