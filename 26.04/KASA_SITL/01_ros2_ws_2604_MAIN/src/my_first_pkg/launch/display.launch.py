from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf = PathJoinSubstitution([
        FindPackageShare("my_first_pkg"),
        "urdf",
        "simple_robot.urdf"
    ])

    robot_description = {
        "robot_description": Command(["cat ", urdf])
    }

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
        ),
    ])
