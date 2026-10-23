import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pix = get_package_share_directory('auv_perception_pixel')
    aut = get_package_share_directory('sauvc_autonomy')
    home = os.path.expanduser('~')
    model = os.path.join(home, 'gate_models', 'best_yolo11n.pt')
    if not os.path.exists(model):
        model = os.path.join(pix, 'models', 'gate_model.engine')

    def arg(n, d):
        return DeclareLaunchArgument(n, default_value=d)

    on = lambda n: IfCondition(LC(n))

    args = [
        arg('perception', 'true'), arg('panel', 'true'), arg('viewer', 'false'),
        arg('bt', 'false'), arg('sim', 'false'), arg('use_video', 'false'),
        arg('video', os.path.join(home, 'test_footage', 'arena-2017.jpg')),
        arg('camera_topic', '/camera/image_raw'), arg('model_path', model),
        arg('hfov_deg', '80.0'), arg('x_sign', '1.0'), arg('image_width', '0.0'),
    ]

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pix, 'launch', 'perception.launch.py')),
        launch_arguments={'camera_topic': LC('camera_topic'),
                          'model_path': LC('model_path')}.items(),
        condition=on('perception'))

    bridge = Node(package='sauvc_autonomy', executable='gate_bridge_pixel.py',
                  name='gate_bridge_pixel', output='screen',
                  respawn=True, respawn_delay=2.0,
                  parameters=[{'hfov_deg': ParameterValue(LC('hfov_deg'), value_type=float),
                               'x_sign': ParameterValue(LC('x_sign'), value_type=float),
                               'image_width': ParameterValue(LC('image_width'), value_type=float)}],
                  condition=on('perception'))

    video = Node(package='image_publisher', executable='image_publisher_node',
                 name='image_publisher', output='log',
                 arguments=[LC('video')],
                 remappings=[('image_raw', LC('camera_topic'))],
                 parameters=[{'publish_rate': 15.0}],
                 condition=on('use_video'))

    panel = Node(package='sauvc_autonomy', executable='gate_panel.py', name='gate_panel',
                 output='log', condition=on('panel'))

    viewer = Node(package='rqt_image_view', executable='rqt_image_view',
                  arguments=['/gate/panel'], condition=on('viewer'))

    bt = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(aut, 'launch', 'sauvc_bt_live.launch.py')),
        condition=on('bt'))

    # Fake vehicle: odometry + IMU + setpoint follower. Its own gate is parked far behind
    # the vehicle so the bridge is the only gate source.
    sim = Node(package='sauvc_autonomy', executable='mock_vehicle_sim', name='mock_vehicle_sim',
               output='screen', parameters=[{'gate_x': -100.0}], condition=on('sim'))

    return LaunchDescription(args + [perception, bridge, panel, viewer, bt, sim,
                                     TimerAction(period=5.0, actions=[video])])
