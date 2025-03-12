import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command,LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
 
def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_share = FindPackageShare(package='custom_robot_sim').find('custom_robot_sim')
    
    gz_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': '-r empty.sdf'
            }.items(),
    )


    model_path=os.path.join(pkg_share,'urdf/my_robot.urdf.xacro')

    urdf_model = LaunchConfiguration('urdf_model')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=model_path, 
        description='Absolute path to robot urdf file'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_model])}]
    )

    entity_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world','empty',
            '-topic','robot_description'
            ]
    )

    ros_gz_bridge_node= Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',

            '/base_to_link1_control@std_msgs/msg/Float64@gz.msgs.Double',
            '/link1_to_link2_control@std_msgs/msg/Float64@gz.msgs.Double',
        ]
    )

    ld = LaunchDescription()
   
    ld.add_action(declare_urdf_model_path_cmd)

    # Add any actions
    ld.add_action(gz_sim)
    ld.add_action(robot_state_publisher)
    ld.add_action(entity_spawner)
    ld.add_action(ros_gz_bridge_node)

    return ld