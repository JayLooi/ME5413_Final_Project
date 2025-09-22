import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import SetParameter, Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('jackal_description'),
        'config',
        'extra_sensors_bridge.yaml'
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value='True'),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_topics_bridge',
            output='screen',
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={config_file}'
            ]
        ),

        Node(
            package='ros_gz_image',
            executable='image_bridge',
            output='screen',
            arguments=['/j100_0000/sensors/front_camera/image_raw'],
            remappings=[
                ('/j100_0000/sensors/front_camera/image_raw', 'front_camera/image_raw')
            ]
        )
    ])
