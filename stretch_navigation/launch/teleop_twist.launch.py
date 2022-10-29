from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution

configurable_parameters = [{'name': 'teleop_type',   'default': "2",                'description': "how to teleop (1=keyboard, 2=joystick, or 'none')"},
                           {'name': 'linear',        'default': "0.5",             'description': "linear speed (m/s)"                               },
                           {'name': 'angular',       'default': "0.5",              'description': "angular speed (rad/s)"                            },
                           {'name': 'joystick_port', 'default': "/dev/input/js0",   'description': "joystick USB device name"                         },
                           ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def generate_launch_description():
    keyboard_teleop_params = [
        {
            'speed': LaunchConfiguration('linear'),
            'turn': LaunchConfiguration('angular')
        }
    ]
    
    # Note: Launching (ros2 launch) twist_teleop_keyboard fails, but running (ros2 run) the node separately works
    # Error: termios.error: (25, 'Inappropriate ioctl for device')
    # keyboard_twist_teleop_node = Node(
    #         package='teleop_twist_keyboard',
    #         executable='teleop_twist_keyboard',
    #         parameters=keyboard_teleop_params,
    #         output='screen',
    #         remappings=[('cmd_vel', '/stretch/cmd_vel')],
    #         condition=IfCondition(
    #             PythonExpression([
    #                 LaunchConfiguration('teleop_type'),
    #                 ' == 1'])
    #         )
    # )

    joy_node_params = [
        {
            'dev': LaunchConfiguration('joystick_port'),
            'autorepeat_rate': 20.0,
            'deadzone': 0.05
        }
    ]
    
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=joy_node_params,
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('teleop_type'),
                    ' == 2'])
            )
    )

    joystick_twist_teleop_params = [
        {
            'enable_button': 4, # Front left button
            'axis_linear.x': 4, # Right stick vertical axis
            'axis_angular.yaw': 3, # Right stick horizontal axis
            'scale_linear.x': LaunchConfiguration('linear'),
            'scale_angular': LaunchConfiguration('angular'),
        }
    ]

    joystick_twist_teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=joystick_twist_teleop_params,
            output='screen',
            remappings=[('cmd_vel', '/stretch/cmd_vel')],
            condition=IfCondition(
                PythonExpression([
                    LaunchConfiguration('teleop_type'),
                    ' == 2'])
            )
    )

    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        # keyboard_twist_teleop_node,
        joy_node,
        joystick_twist_teleop_node,]
    )