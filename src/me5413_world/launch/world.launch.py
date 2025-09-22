import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import SetParameter, Node
from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ros_gz_pkg = FindPackageShare('ros_gz_sim')
    clearpath_gz_pkg = FindPackageShare('clearpath_gz')
    world_pkg = FindPackageShare('me5413_world')
    robot_desc_pkg = FindPackageShare('jackal_description')

    world_path = PathJoinSubstitution([world_pkg, 'worlds', 'me5413_project_2425.world'])
    model_path = PathJoinSubstitution([world_pkg, 'models'])
    plugin_path = PathJoinSubstitution([world_pkg, 'plugins'])
    setup_path = PathJoinSubstitution([robot_desc_pkg, 'robot_config'])

    return LaunchDescription([
        SetParameter(name='use_sim_time', value='True'),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH',
                               value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
                                      os.pathsep, model_path,
                                      os.pathsep, os.path.expanduser('~/gazebo_models')]),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH',
                               value=plugin_path),
        # ExecuteProcess(cmd=['echo', '$GZ_SIM_RESOURCE_PATH'], output='screen', shell=True),
        IncludeLaunchDescription(
            PathJoinSubstitution([
                ros_gz_pkg,
                'launch',
                'gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': [world_path, ' -r'],  # specify -r option to start simulation when launch
                'on_exit_shutdown': 'True'
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                clearpath_gz_pkg,
                'launch',
                'robot_spawn.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'world': world_path,
                'setup_path': setup_path,
                'rviz': 'true',
                'x': '0',
                'y': '0',
                'z': '3.0',
                'yaw': '1.57079632679'
            }.items()
        ),

        IncludeLaunchDescription(
            PathJoinSubstitution([
                robot_desc_pkg,
                'launch',
                'gz_to_ros_bridge.launch.py'
            ])
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ]
        )
    ])
