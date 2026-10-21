from explorer_bringup.launch.controller_manager_spawner import (
    declare_node_gripper_controller_spawner,
)
from explorer_bringup.launch.hardware import declare_hardware_node_group
from explorer_bringup.launch.hardware_parameters import (
    declare_hardware_argument_list,
)
from explorer_bringup.launch.simulation import declare_simulation_node_group
from explorer_bringup.launch.simulation_parameters import (
    declare_simulation_argument_list,
)
from explorer_bringup.launch.shared_parameters import (
    get_parameter_debug,
    get_parameter_simulation,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_manager_node(context):
    parameters = [
        LaunchConfiguration('custom_controller_config_file').perform(context)
    ]
    extra_params = LaunchConfiguration('extra_params').perform(context)
    if extra_params:
        parameters.append(extra_params)

    return [Node(
        package='cartesian_manager',
        executable='cartesian_manager_node',
        name='cartesian_manager',
        output='screen',
        parameters=parameters,
    )]


def declare_controller_spawner():
    controller_config = LaunchConfiguration('custom_controller_config_file')

    return Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            '--controller-manager',
            '/controller_manager',
            '--activate-as-group',
            '--controller',
            'explorer_custom_controller',
            '--param-file',
            controller_config,
            [
                '--controller-ros-args=--ros-args '
                '-p simulation:=',
                get_parameter_simulation(),
                ' -p debug:=',
                get_parameter_debug(),
            ],
            '--controller',
            'qontrol_explorer',
            '--param-file',
            controller_config,
        ],
    )


def generate_launch_description():
    robot_controller_config = 'custom_controller'
    joystick_config_file = LaunchConfiguration('joystick_config_file')

    declared_arguments = [
        *declare_simulation_argument_list(
            robot_controller_config=robot_controller_config,
        ),
        *declare_hardware_argument_list(
            robot_controller_config=robot_controller_config,
        ),
        DeclareLaunchArgument(
            'joystick_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('joystick_mapper'),
                'config',
                'joystick_3d.yaml',
            ]),
            description='Joystick mapper parameter file.',
        ),
        DeclareLaunchArgument(
            'custom_controller_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('cartesian_manager'),
                'config',
                'explorer_params.yaml',
            ]),
            description=(
                'Shared parameter file for cartesian_manager, '
                'explorer_custom_controller, and qontrol_explorer.'
            ),
        ),
        DeclareLaunchArgument(
            'extra_params',
            default_value='',
            description='Parameter file layered after explorer_params.yaml for the manager.',
        ),
    ]

    robot_controller_list = [
        declare_controller_spawner(),
        declare_node_gripper_controller_spawner(output='screen'),
    ]

    robot_simulation = declare_simulation_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=False,
    )

    robot_hardware = declare_hardware_node_group(
        robot_controller_list=robot_controller_list,
        launch_qp_solving=False,
        robot_controller_config_type=robot_controller_config,
    )

    manager_node = OpaqueFunction(
        function=launch_manager_node,
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
    )

    joystick_mapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('joystick_mapper'),
                'launch',
                'joystick_mapper.launch.py',
            ])
        ),
        launch_arguments={
            'config_file': joystick_config_file,
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        robot_simulation,
        robot_hardware,
        manager_node,
        joy_node,
        joystick_mapper_launch,
    ])
