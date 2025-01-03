from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_cpp = Node(
        package="susy_controller",
        executable="noisy_controller",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
    )

    return [
        noisy_controller_cpp,
    ]



def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius", 
        default_value="0.128",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.84",
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.004",
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.001",
    )


    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["susy_controller", 
                   "--controller-manager", 
                   "/controller_manager"
        ]
    )

    simple_controller_node = Node(
        package="susy_controller",
        executable="simple_controller",
        parameters=[{
            "wheel_radius": wheel_radius,
            "wheel_separation": wheel_separation, 
            "use_sim_time": use_sim_time,
        }]
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        wheel_controller_spawner,
        simple_controller_node,
        noisy_controller_launch
    ])