import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


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


def generate_launch_description():
    # planning_context
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("stretch_moveit_config"),
            "config",
            "stretch.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        "stretch_moveit_config", "config/stretch_description.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml("stretch_moveit_config", "config/kinematics.yaml")

    # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("stretch_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    pick_place_demo = Node(
        package="pick_place_task",
        executable="pick_place_task_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
        ],
    )

    return LaunchDescription([pick_place_demo])
