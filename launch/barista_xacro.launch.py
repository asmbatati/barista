import os
import random
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    #files
    description_package_name = "barista_robot_description"
    xacro_file='barista_robot_model.urdf.xacro'
    rviz_file = 'xacro_vis.rviz'
    world_selected = 'barista_empty.world'
    # Position and orientation
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_base_name = "barista_robot"
    #fetching
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_robot_gazebo = get_package_share_directory(description_package_name)
    install_dir = get_package_prefix(description_package_name)

    #instalation
    gazebo_models_path = os.path.join(pkg_robot_gazebo, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    print("\nGAZEBO MODELS PATH==" + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH==" + str(os.environ["GAZEBO_PLUGIN_PATH"]) + '\n')

    # launch argument for the world file
    world_file_arg=DeclareLaunchArgument('world', default_value=[os.path.join(pkg_robot_gazebo, 'worlds', world_selected), ''], description='Path to the Gazebo world file')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Robot State Publisher
    robot_desc_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_path = os.path.join(robot_desc_path, 'xacro', xacro_file)
    robot_name='Morty'
    include_laser='true'
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_name,
        emulate_tty=True,
        parameters=[{ 'use_sim_time': True,
                     'robot_description': Command(['xacro ', xacro_path, ' robot_name:=', robot_name, ' include_laser:=', include_laser])}],
        output='screen'
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    # Spawn ROBOT Set Gazebo
    entity_name = robot_base_name + "-" + str(int(random.random() * 100000))
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
                   '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
                   '-topic', robot_name+'/robot_description']
    )

    return LaunchDescription([
        world_file_arg,
        gazebo,
        robot_state_publisher_node,
        rviz_node,
        spawn_robot,
    ])