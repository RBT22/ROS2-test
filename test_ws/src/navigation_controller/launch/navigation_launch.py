import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
   
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value=TextSubstitution(text="True")
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "tb3_simulation_launch.py",
            )
        ),
        launch_arguments={
            "params_file": os.path.join(
                get_package_share_directory("navigation_controller"),
                "config",
                "nav2_params_mod.yaml",
            ),
            "use_rviz": LaunchConfiguration("use_rviz"),
            # 'headless': 'True',
        }.items(),
    )

    navigator = Node(
        package="navigation_controller",
        executable="navigator",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz_arg)

    ld.add_action(nav2_bringup)
    ld.add_action(navigator)

    return ld
