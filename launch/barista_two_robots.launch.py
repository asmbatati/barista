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
    rviz_file = 'mrs_vis.rviz'
    world_selected = 'barista_empty.world'
    # Position and orientation
    Morty_position = [1.0, 1.0, 0.2]
    Morty_orientation = [0.0, 0.0, 0.0]
    Rick_position = [-1.0, -1.0, 0.2]
    Rick_orientation = [0.0, 0.0, 0.0]
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

    # Morty State Publisher
    robot_desc_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_path = os.path.join(robot_desc_path, 'xacro', xacro_file)
    robot1_name='morty'
    include_laser='true'
    robot1_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot1_name,
        emulate_tty=True,
        parameters=[{ 'frame_prefix': robot1_name+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', xacro_path, ' robot_name:=', robot1_name, ' include_laser:=', include_laser, " robot_color:=", "Blue"])}],
        output='screen'
    )

    # Rick State Publisher
    robot_desc_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_path = os.path.join(robot_desc_path, 'xacro', xacro_file)
    robot2_name='rick'
    include_laser='true'
    robot2_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot2_name,
        emulate_tty=True,
        parameters=[{ 'frame_prefix': robot2_name+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', xacro_path, ' robot_name:=', robot2_name, ' include_laser:=', include_laser, " robot_color:=", "Red"])}],
        output='screen'
    )

    # Spawn Morty
    # entity_name = robot_base_name + "-" + str(int(random.random() * 100000))
    entity_name = robot1_name
    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
                   '-x', str(Morty_position[0]), '-y', str(Morty_position[1]), '-z', str(Morty_position[2]),
                   '-R', str(Morty_orientation[0]), '-P', str(Morty_orientation[1]), '-Y', str(Morty_orientation[2]),
                   '-topic', robot1_name+'/robot_description']
    )

    # Spawn Rick
    # entity_name = robot_base_name + "-" + str(int(random.random() * 100000))
    entity_name = robot2_name
    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
                   '-x', str(Rick_position[0]), '-y', str(Rick_position[1]), '-z', str(Rick_position[2]),
                   '-R', str(Rick_orientation[0]), '-P', str(Rick_orientation[1]), '-Y', str(Rick_orientation[2]),
                   '-topic', robot2_name+'/robot_description']
    )

    static_tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot1_name+'/odom']
    )
    static_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', robot2_name+'/odom']
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

    return LaunchDescription([
        world_file_arg,
        gazebo,
        robot1_state_publisher_node,
        robot2_state_publisher_node,
        spawn_robot1,
        spawn_robot2,
        static_tf1,
        static_tf2,
        rviz_node,
    ])