
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable, LaunchConfiguration 
from launch_ros.actions import Node


def generate_launch_description():
    imu_receiver_node = Node(
        package='imu_receiver',
        executable='udp_ros2_receiver',
        name='udp_ros2_receiver',
    )
    configure_fv_controller =  ExecuteProcess(
        cmd=[
            [
            'ros2 control load_controller forward_velocity_controller --set-state inactive && ros2 control switch_controllers --activate forward_velocity_controller --deactivate joint_trajectory_controller',
            ]
        ],
        shell=True,
    )
    return LaunchDescription(
    [
        configure_fv_controller,
        imu_receiver_node, 
    ]
    )