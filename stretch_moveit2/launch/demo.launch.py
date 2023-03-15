from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    moveit_config_path = get_package_share_path('stretch_moveit2')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('pipeline', default_value='ompl', description='specify the planning pipeline'))
    ld.add_action(DeclareLaunchArgument('db', default_value='false', choices=['true', 'false'],
                                        description='By default, we do not start a database (it can be large)'))

    ld.add_action(DeclareLaunchArgument('db_path', default_value=str(moveit_config_path / 'default_warehouse_mongo_db'),
                                        description='Allow user to specify database location'))
    ld.add_action(DeclareLaunchArgument('debug', default_value='false', choices=['true', 'false'],
                                        description='By default, we are not in debug mode'))

    # MoveIt's "demo" mode replaces the real robot driver with the joint_state_publisher.
    # The latter one maintains and publishes the current joint configuration of the simulated robot.
    # It also provides a GUI to move the simulated robot around "manually".
    # This corresponds to moving around the real robot without the use of MoveIt.
    ld.add_action(DeclareLaunchArgument('use_gui', default_value='false', choices=['true', 'false'],
                                        description="By default, hide joint_state_publisher's GUI"))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false']))

    # Load the URDF, SRDF and other .yaml configuration files
    robot_description_content = Command(
        ['xacro ', str(moveit_config_path / 'config' / 'stretch.xacro')]
    )
    with open(moveit_config_path / 'config/stretch_description.srdf', 'r') as f:
        semantic_content = f.read()

    # If needed, broadcast static tf for robot root
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'])
    ld.add_action(static_tf)

    # Given the published joint states, publish tf for the robot links
    rsp_node = Node(package='robot_state_publisher', executable='robot_state_publisher',
                    respawn=True, output='screen',
                    parameters=[{'robot_description': robot_description_content, 'publish_frequency': 15.0}])
    ld.add_action(rsp_node)

    # Run the main MoveIt executable without trajectory execution (we do not have controllers configured by default)
    move_group_launch_py = PythonLaunchDescriptionSource(str(moveit_config_path / 'launch/move_group.launch.py'))
    move_group_launch_args = {
        'allow_trajectory_execution': 'true',
        'fake_execution': 'true',
        'info': 'true',
        'debug': LaunchConfiguration('debug'),
        'pipeline': LaunchConfiguration('pipeline'),
        'robot_description': robot_description_content,
        'semantic_config': semantic_content,
    }
    move_group_launch = IncludeLaunchDescription(move_group_launch_py,
                                                 launch_arguments=move_group_launch_args.items())
    ld.add_action(move_group_launch)

    # Run Rviz and load the default config to see the state of the move_group node
    moveit_rviz_launch_py = PythonLaunchDescriptionSource(
        str(moveit_config_path / 'launch/moveit_rviz.launch.py')
    )
    moveit_rviz_args = {
        'rviz_config': str(moveit_config_path / 'launch/moveit.rviz'),
        'debug': LaunchConfiguration('debug'),
        'robot_description': robot_description_content,
        'semantic_config': semantic_content,
    }
    moveit_rviz_launch = IncludeLaunchDescription(moveit_rviz_launch_py, launch_arguments=moveit_rviz_args.items(),
                                                  condition=IfCondition(LaunchConfiguration('use_rviz')))
    ld.add_action(moveit_rviz_launch)

    # If database loading was enabled, start mongodb as well
    warehouse_launch_py = PythonLaunchDescriptionSource(
        str(moveit_config_path / 'launch/warehouse_db.launch.py')
    )
    warehouse_launch = IncludeLaunchDescription(warehouse_launch_py,
                                                launch_arguments={'moveit_warehouse_database_path':
                                                                  LaunchConfiguration('db_path')}.items(),
                                                condition=IfCondition(LaunchConfiguration('db'))
                                                )
    ld.add_action(warehouse_launch)

    # Fake joint driver
    fake_joint_driver_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description_content},
                    str(moveit_config_path / 'config/ros_controllers.yaml'),
                    ],
    )
    ld.add_action(fake_joint_driver_node)

    spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['stretch_controller'],
    )
    ld.add_action(spawner_node)

    spawner_node2 = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'],
    )
    ld.add_action(spawner_node2)

    return ld
