# command for teleop
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/Morty/cmd_vel

import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

# this is the function launch  system will look for
def generate_launch_description():

    ld = LaunchDescription()

    ####### DATA INPUT ##########
    xacro_file = "barista_robot_model.urdf.xacro"
    package_description = "barista_robot_description"
    ####### DATA INPUT END ##########

    print("Fetching URDF ==>")

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_barista_gazebo = get_package_share_directory(package_description)

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = package_description
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_barista_gazebo package
    gazebo_models_path = os.path.join(pkg_barista_gazebo, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )   

    # Robot State Publisher
    robot_desc_path = os.path.join(get_package_share_directory(description_package_name))
    robot_xacro_path = os.path.join(robot_desc_path, 'xacro', xacro_file)
    robot_name='Morty'
    include_laser='true'
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace=robot_name,
        emulate_tty=True,
        parameters=[{ 'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_xacro_path, ' robot_name:=', robot_name, ' include_laser:=', include_laser])}],
        output='screen'
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'xacro_vis.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # Spawn robot in Gazebo
    position = [0.0, 0.0, 0.2]
    orientation = [0.0, 0.0, 0.0]
    robot_base_name = "barista_robot"
    entity_name = robot_base_name+"-"+str(int(random.random()*100000))
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    # create and return launch description object
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gazebo)
    ld.add_action(rviz_node)
    ld.add_action(spawn_robot)

    return ld