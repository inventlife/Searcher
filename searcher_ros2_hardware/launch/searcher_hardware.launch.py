from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # URDF path
    urdf_path = os.path.join(
        get_package_share_directory("searcher_description"),
        "urdf",
        "searcher.urdf.xacro"
    )

    # Declare robot_description argument
    robot_description_arg = DeclareLaunchArgument(
        "robot_description",
        default_value=Command(["xacro ", urdf_path]),
        description="Robot description (URDF) for the robot_state_publisher"
    )

    # Launch robot_state_publisher to publish ~/robot_description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": LaunchConfiguration("robot_description")
        }],
        output="screen"
    )

    return LaunchDescription([
        robot_description_arg,
    #    robot_state_publisher,
    ])
