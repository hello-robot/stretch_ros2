import os
import xacro
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


# Mapping from entry in the stretch_body configuration to joints in the SRDF
CONFIGURATION_TRANSLATION = {
    'lift': ['lift'],
    'wrist_yaw': ['wrist_yaw'],
    'stretch_gripper': ['joint_gripper_finger_left', 'joint_gripper_finger_right'],
    'base': ['position/x', 'position/theta'],
    'arm': ['joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'],
    'head': ['joint_head_pan', 'joint_head_tilt']
}

def load_joint_limits_from_config(mode='default'):
    """Translate the values from the robot configuration to params to be used by MoveIt."""
    params = {'joint_limits': {}}
    try:
        from stretch_body.device import Device
        d = Device()
        for config_name, joint_names in CONFIGURATION_TRANSLATION.items():
            config = d.robot_params[config_name]
            config_limits = config['motion'].get(mode, {})
            result = {}
            for name, abrev in [('velocity', 'vel'), ('acceleration', 'accel')]:
                cfg_name = f'{abrev}_m'
                if cfg_name in config_limits:
                    result[f'has_{name}_limits'] = True
                    result[f'max_{name}'] = config_limits[cfg_name]
                else:
                    result[f'has_{name}_limits'] = False
            for joint_name in joint_names:
                params['joint_limits'][joint_name] = dict(result)
    except (KeyError, ModuleNotFoundError):
        # We may reach here if HELLO_FLEET_ID or HELLO_FLEET_PATH is not set
        # or stretch_body.device is not on the PYTHONPATH
        # in which case we load the defaults
        print('Load from default')
        return load_yaml('stretch_moveit_config', 'config/default_joint_limits.yaml')
    return params

def generate_launch_description():
    # planning_context
    robot_description_config = xacro.process_file(os.path.join(get_package_share_directory('stretch_moveit_config'),
                                                            'config',
                                                            'stretch.xacro'),
    )
    robot_description = {'robot_description' : robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "stretch_moveit_config", "config/stretch_description.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "stretch_moveit_config", "config/kinematics.yaml"
    )

    joint_limits_yaml = {'robot_description_planning': load_joint_limits_from_config()}

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_demo",
        package="stretch_roscon_demos",
        executable="move_group_interface_demo",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, joint_limits_yaml],
    )

    return LaunchDescription([move_group_demo])