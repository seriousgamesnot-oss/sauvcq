import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('sauvc_autonomy')
    return LaunchDescription([
        Node(
            package='sauvc_autonomy',
            executable='sauvc_bt_executor',
            name='sauvc_bt_executor',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'bt_config.yaml'),
                {'bt_xml_path': os.path.join(share, 'bt_trees', 'SAUVC_Qualification_Main.xml')},
            ],
        ),
    ])
