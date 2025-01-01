import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    susy_description_dir = get_package_share_directory('susy_description')
    susy_description_share = os.path.join(get_package_prefix('susy_description'), 'share')
    #gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
        susy_description_dir, 'urdf', 'susy2.urdf.xacro'
        )
    )

    world_name_arg = DeclareLaunchArgument(name = "world_name", default_value="empty")
    world_path = PathJoinSubstitution([
        susy_description_dir,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"),"'","+ '.world'"])
    ])

    model_path = str(Path(susy_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("susy_description"), 'models')
    

    env_variable = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", model_path
    )

    
    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            "True"
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "susy"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )
    return LaunchDescription([
        env_variable,
        model_arg,
        world_name_arg,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
    ])
