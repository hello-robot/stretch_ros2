import argparse
import yaml

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    stretch_moveit2_path = get_package_share_path('stretch_moveit2')
    stretch_description_path = get_package_share_path('stretch_description')

    # Load config
    robot_description_urdf = Command(['xacro ', str(stretch_description_path / 'urdf' / 'stretch_description.xacro')])
    robot_description_semantic_config = load_config('stretch_description.srdf')
    joint_limits_config = load_joint_limits()
    moveit_kinematics_config = load_config('moveit_kinematics.yaml')
    sensors_config = load_config('sensors_3d.yaml')
    ompl_planning_pipeline_config = load_config('planning_ompl.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': load_config('moveit_simple_controllers.yaml'),
                          'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}
    planning_scene_monitor_parameters = {'publish_planning_scene': True,
                                         'publish_geometry_updates': True,
                                         'publish_state_updates': True,
                                         'publish_transforms_updates': True}

    # Launch args
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='True', choices=['True', 'False'],
        description='Whether to show Rviz'
    )

    declare_rviz_cfg_fpath_arg = DeclareLaunchArgument(
        'rviz_cfg_fpath',
        default_value=str(stretch_moveit2_path / 'rviz' / 'moveit.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_fake_arg = DeclareLaunchArgument(
        'fake',
        default_value='False', choices=['True', 'False'],
        description='Performs fake motion planning/execution on a ros2_control backed fake robot'
    )

    # Nodes
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     arguments=['-d', LaunchConfiguration('rviz_cfg_fpath')],
                     condition=IfCondition(LaunchConfiguration("rviz")),
                     parameters=[{'robot_description': robot_description_urdf},
                                 {'robot_description_semantic': robot_description_semantic_config},
                                 ompl_planning_pipeline_config,
                                 moveit_kinematics_config])

    moveit_node = Node(package='moveit_ros_move_group',
                       executable='move_group',
                       parameters=[{'robot_description': robot_description_urdf},
                                   {'robot_description_semantic': robot_description_semantic_config},
                                   moveit_kinematics_config,
                                   sensors_config,
                                   joint_limits_config,
                                   ompl_planning_pipeline_config,
                                   trajectory_execution,
                                   moveit_controllers,
                                   planning_scene_monitor_parameters])

    fake_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(stretch_moveit2_path), '/launch/fake_driver.launch.py']),
        launch_arguments={'rviz': 'False'}.items(),
        condition=IfCondition(LaunchConfiguration("fake"))
    )

    return LaunchDescription([declare_rviz_arg,
                              declare_rviz_cfg_fpath_arg,
                              declare_fake_arg,
                              rviz_node,
                              moveit_node,
                              fake_driver_launch])


def load_config(filename: str):
    stretch_moveit2_path = get_package_share_path('stretch_moveit2')
    filepath = stretch_moveit2_path / 'config' / filename
    with open(filepath.absolute(), 'r') as stream:
        if filepath.suffix == '.yaml':
            return yaml.load(stream, Loader=yaml.FullLoader)
        elif filepath.suffix == '.srdf':
            return stream.read()


def load_joint_limits(mode='default'):
    """Translate the joint limits from Stretch Body to params to be used by MoveIt.
    """
    params = {'joint_limits': {}}
    try:
        # from stretch_body.device import Device
        # d = Device()
        # for config_name, joint_names in CONFIGURATION_TRANSLATION.items():
        #     config = d.robot_params[config_name]
        #     config_limits = config['motion'].get(mode, {})
        #     result = {}
        #     for name, abrev in [('velocity', 'vel'), ('acceleration', 'accel')]:
        #         if abrev in config_limits:
        #             result[f'has_{name}_limits'] = True
        #             result[f'max_{name}'] = config_limits[abrev]
        #         elif f'{abrev}_m' in config_limits:
        #             result[f'has_{name}_limits'] = True
        #             result[f'max_{name}'] = config_limits[f'{abrev}_m']
        #         else:
        #             result[f'has_{name}_limits'] = False
        #     for joint_name in joint_names:
        #         params['joint_limits'][joint_name] = dict(result)
        params = load_config('default_joint_limits.yaml') # TODO: implement above
    except (KeyError, ModuleNotFoundError):
        # We may reach here if HELLO_FLEET_ID or HELLO_FLEET_PATH is not set
        # or stretch_body.device is not on the PYTHONPATH
        # in which case we load the defaults
        params = load_config('default_joint_limits.yaml')

    return params
