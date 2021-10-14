from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_prefix = get_package_share_directory('stretch_core')

    driver_py = PythonLaunchDescriptionSource([package_prefix, '/launch/stretch_driver.launch.py'])
    driver_launch = IncludeLaunchDescription(driver_py, launch_arguments={'broadcast_odom_tf': 'true'}.items())

    resolution_arg = DeclareLaunchArgument(
        'high_res',
        default_value='False',
        description='Whether to use high resolution realsense'
    )

    hi_realsense_py = PythonLaunchDescriptionSource([package_prefix, '/launch/d435i_high_resolution.launch.py'])
    hi_realsense_launch = IncludeLaunchDescription(hi_realsense_py,
                                                   condition=IfCondition(LaunchConfiguration('high_res')))

    lo_realsense_py = PythonLaunchDescriptionSource([package_prefix, '/launch/d435i_low_resolution.launch.py'])
    lo_realsense_launch = IncludeLaunchDescription(lo_realsense_py,
                                                   condition=UnlessCondition(LaunchConfiguration('high_res')))

    rplidar_py = PythonLaunchDescriptionSource([package_prefix, '/launch/rplidar.launch.py'])
    rplidar_launch = IncludeLaunchDescription(rplidar_py)

    # TODO: Include respeaker
    # TODO: Include ekf

    return LaunchDescription([
        driver_launch,
        resolution_arg,
        hi_realsense_launch,
        lo_realsense_launch,
        rplidar_launch
    ])
