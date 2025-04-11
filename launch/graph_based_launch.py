from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    share_dir = get_package_share_directory('graph_based_planning_pkg')
    share_lio_dir = get_package_share_directory('lio_sam')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'rviz.rviz')
    parameter_file_path = os.path.join(share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        # clustering_node
        Node(
            package='graph_based_planning_pkg',
            executable='clustering_node',
            name='clustering_node',
            parameters=[parameter_file_path],
            output="screen"
        ),

        # path_planning_node
        Node(
            package='graph_based_planning_pkg',
            executable='path_planning_node',
            name='path_planning_node',
            parameters=[parameter_file_path],
            output="screen"

        ),

        # row_detection_node
        Node(
            package='graph_based_planning_pkg',
            executable='row_detection_node',
            name='row_detection_node',
            parameters=[parameter_file_path],
            output="screen"

        ),

        # path_spline_node
        Node(
            package='graph_based_planning_pkg',
            executable='path_spline_node',
            name='path_spline_node',
            parameters=[parameter_file_path],
            output="screen"

        ),

        # RViz 실행
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
        ),

        # lio_sam 런치 포함
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_lio_dir, 'launch', 'run.launch.py')
            )
        )
    ])
