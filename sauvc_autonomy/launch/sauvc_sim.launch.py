import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('sauvc_autonomy')
    return LaunchDescription([
        # TEST values only: replace with real pool numbers for any real run
        DeclareLaunchArgument('pool_depth', default_value='3.0'),
        DeclareLaunchArgument('min_clearance', default_value='0.5'),
        DeclareLaunchArgument('safe_depth', default_value='0.6'),
        Node(package='sauvc_autonomy', executable='mock_vehicle_sim', output='screen'),
        Node(package='sauvc_autonomy', executable='sauvc_bt_executor', output='screen',
             parameters=[os.path.join(share, 'config', 'bt_config.yaml'), {
                 'bt_xml_path': os.path.join(share, 'bt_trees', 'SAUVC_Qualification_Main.xml'),
                 'floor_topic': '/varuna/floor_clearance',
                 'pool_depth_m': LaunchConfiguration('pool_depth'),
                 'min_floor_clearance_m': LaunchConfiguration('min_clearance'),
                 'floor_safe_depth_m': LaunchConfiguration('safe_depth'),
             }]),
    ])
