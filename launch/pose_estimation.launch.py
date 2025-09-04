import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    pkg_human_pose_detector = get_package_share_directory('human_pose_detector')
    rviz_config_file = os.path.join(pkg_human_pose_detector, 'config', 'pose_estimation.rviz')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Se verdadeiro (true), inicia o RViz2 com a configuração salva.'
    )

    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_link_broadcaster',
        arguments=['0', '0', '0.50',  # <-- MEÇA E AJUSTE ESSES VALORES (X, Y, Z em metros)
                   '0', '0', '0',     # <-- MEÇA E AJUSTE ESSES VALORES (Yaw, Pitch, Roll em radianos)
                   'base_link',
                   'camera_link']
    )

    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')]),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'depth_module.emitter_enabled': 'false'
        }.items()
    )

    pose_tf_publisher_node = Node(
        package='human_pose_detector',
        executable='pose_tf_publisher',
        name='pose_tf_publisher_node',
        output='screen'
    )
    
    goal_sender_node = Node(
        package='real_robotino_pkg',
        executable='goal_sender.py',
        name='goal_sender_node',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        static_tf_publisher_node,
        realsense_camera_node,
        pose_tf_publisher_node,
        goal_sender_node,
        rviz_node,
    ])