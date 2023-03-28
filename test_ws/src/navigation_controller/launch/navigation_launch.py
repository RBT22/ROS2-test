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
            "use_rviz": LaunchConfiguration("use_rviz"),
            # 'headless': 'True',
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(use_rviz_arg)

    ld.add_action(nav2_bringup)

    return ld
