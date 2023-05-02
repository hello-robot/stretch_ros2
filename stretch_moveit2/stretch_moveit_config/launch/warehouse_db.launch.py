from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    moveit_config_path = get_package_share_path('stretch_moveit_config')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('reset', default_value='false', choices=['true', 'false']))

    # If not specified, we'll use a default database location
    ld.add_action(DeclareLaunchArgument('moveit_warehouse_database_path',
                                        default=str(moveit_config_path / 'default_warehouse_mongo_db')))

    # The default DB port for moveit (not default MongoDB port to avoid potential conflicts)
    ld.add_action(DeclareLaunchArgument('moveit_warehouse_port', default='33829'))

    # The default DB host for moveit
    ld.add_action(DeclareLaunchArgument('moveit_warehouse_host', default='localhost'))

    # Load warehouse parameters
    db_parameters = [
        {'overwrite': False,
         'database_path': LaunchConfiguration('moveit_warehouse_database_path'),
         'warehouse_port': LaunchConfiguration('moveit_warehouse_port'),
         'warehouse_host': LaunchConfiguration('moveit_warehouse_host'),
         'warehouse_exec': 'mongod',
         'warehouse_plugin': 'warehouse_ros_mongo::MongoDatabaseConnection'
         },
    ]
    # Run the DB server
    db_node = Node(package='warehouse_ros_mongo',
                   executable='mongo_wrapper_ros.py',
                   cwd="ROS_HOME",
                   parameters=db_parameters,
                   )
    ld.add_action(db_node)

    # If we want to reset the database, run this node
    reset_node = Node(package='moveit_ros_warehouse',
                      executable='moveit_init_demo_warehouse',
                      output='screen',
                      condition=IfCondition(LaunchConfiguration('reset')),
                      )
    ld.add_action(reset_node)

    return ld
