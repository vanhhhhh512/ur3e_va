# Khoi dong moi truong mo phong: Ignition Gazebo (UR3e + bang ve) -> ros2_control -> MoveIt -> RViz.
# RViz mo bang file cau hinh rieng trong rviz/ de co san display Marker cho net muc.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

UR_TYPE = "ur3e"
DESCRIPTION_PACKAGE = "ur_pen_plotter"
DESCRIPTION_FILE = "ur3e_pen.urdf.xacro"


def generate_launch_description():
    package = FindPackageShare(DESCRIPTION_PACKAGE)
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    launch_rviz = LaunchConfiguration("launch_rviz")
    world_file = LaunchConfiguration("world_file")

    # Boc moi include trong GroupAction (co scope rieng). Neu khong, launch_arguments
    # truyen xuong include se GHI DE bien cung ten o day: ca hai include deu nhan
    # launch_rviz:=false nen RViz cua rieng minh ben duoi se khong bao gio chay.
    gz_control = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch", "/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": UR_TYPE,
            "description_package": DESCRIPTION_PACKAGE,
            "description_file": DESCRIPTION_FILE,
            "runtime_config_package": DESCRIPTION_PACKAGE,
            "controllers_file": "sim_controllers.yaml",
            "initial_joint_controller": "joint_trajectory_controller",
            "world_file": world_file,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
        }.items(),
    )])

    moveit = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": UR_TYPE,
            "description_package": DESCRIPTION_PACKAGE,
            "description_file": DESCRIPTION_FILE,
            "use_sim_time": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="pen_plotter_rviz",
        output="screen",
        arguments=["-d", PathJoinSubstitution([package, "rviz", "pen_plotter.rviz"])],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gazebo_gui", default_value="true", description="Mo cua so Ignition Gazebo?"
            ),
            DeclareLaunchArgument(
                "launch_rviz", default_value="true", description="Mo RViz de xem net muc?"
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value=PathJoinSubstitution([package, "worlds", "pen_board.sdf"]),
                description="File the gioi SDF (mac dinh: nen + bang ve dung tai x=0.40 m).",
            ),
            gz_control,
            moveit,
            rviz,
        ]
    )
