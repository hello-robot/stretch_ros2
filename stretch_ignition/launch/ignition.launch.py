import os
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    rviz_arg = LaunchConfiguration('rviz', default=False)
    aws_arg = LaunchConfiguration('aws', default=False)
    nav_arg = LaunchConfiguration('nav', default=True)

    robot_description_path =  os.path.join(get_package_share_directory("stretch_description"), "urdf", "stretch_ignition.xacro")
    robot_description_config = xacro.process_file(
        robot_description_path
    )
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Ignition gazebo
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_stretch_ignition = get_package_share_directory('stretch_ignition')
    # empty_world_str = "-r " + os.path.join(pkg_stretch_ignition, 'worlds', 'empty_world.sdf')
    empty_world_str = "-r " + os.path.join(pkg_stretch_ignition, 'worlds', 'pick_place_world.sdf')
    empty_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': empty_world_str}.items(),
        condition=UnlessCondition(aws_arg)
    )

    aws_world_str = "-r " + os.path.join(pkg_stretch_ignition, 'worlds', 'aws_small_house.sdf')
    aws_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': aws_world_str}.items(),
        condition=IfCondition(aws_arg)
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_stretch_ignition, 'rviz', 'stretch_ignition.rviz')],
        condition=IfCondition(rviz_arg)
    )

    # Spawn from xacro
    # spawn = Node(package='ros_ign_gazebo', executable='create',
    #             arguments=[
    #                 '-name', 'stretch',
    #                 '-topic', 'robot_description',
    #                 '-z', '0.1',
    #                 ],
    #             output='screen',
    #             )

    # Spawn SDF
    stretch_sdf_path =  os.path.join(get_package_share_directory("stretch_description"), "urdf", "stretch_ignition.sdf")
    spawn = Node(package='ros_ign_gazebo', executable='create',
                arguments=[
                    '-name', 'stretch',
                    '-file', stretch_sdf_path,
                    '-z', '0.01',
                    ],
                output='screen',
                )

    # Ign Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
                # Velocity commands (ROS2 -> IGN)
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                # JointTrajectory bridge (ROS2 -> IGN)
                '/joint_trajectory@trajectory_msgs/msg/JointTrajectory]ignition.msgs.JointTrajectory',
                # Odometry (IGN -> ROS2)
                # '/model/stretch/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                # odom->base_link tf (IGN -> ROS2)
                '/model/stretch/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/world/default/model/stretch/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # JointTrajectoryProgress bridge (IGN -> ROS2)
                '/joint_trajectory_progress@std_msgs/msg/Float32[ignition.msgs.Float',
                # Lidar (IGN -> ROS2)
                '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                # Base IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Base Magnetometer (IGN -> ROS2)
                '/magnetometer@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer',
                # Wrist Accelerometer (IGN -> ROS2)
                '/wrist_imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Realsense IMU
                '/realsense_imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                # Realsense Camera (TODO: Port realsense plugin when ign supports custom plugins) (IGN -> ROS2)
                # Realsense Color
                # '/world/default/model/stretch/link/camera_color_optical_frame/sensor/realsense_d435_color/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                # '/world/default/model/stretch/link/camera_color_optical_frame/sensor/realsense_d435_color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                # Realsense Depth
                '/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
                '/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
                '/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
                '/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/image@sensor_msgs/msg/Image[ignition.msgs.Image'
                # Realsense IR 1
                # '/world/default/model/stretch/link/camera_infra1_optical_frame/sensor/realsense_d435_ir/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                # '/world/default/model/stretch/link/camera_infra1_optical_frame/sensor/realsense_d435_ir/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',                
                # Realsense IR 2
                # '/world/default/model/stretch/link/camera_infra2_optical_frame/sensor/realsense_d435_ir2/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                # '/world/default/model/stretch/link/camera_infra2_optical_frame/sensor/realsense_d435_ir2/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',  
                ],
        remappings=[
            ("/model/stretch/tf", "tf"),
            ("/world/default/model/stretch/joint_state", "joint_states"),
            # ("/odom", "position"),
            ("/imu", "imu/data"),
            ("/magnetometer", "mag"),
            ("/wrist_imu", "wrist_imu/data"),
            # Realsense IMU
            ("/realsense_imu", "realsense/imu/data"),
            # Realsense Color
            # ("/world/default/model/stretch/link/camera_color_optical_frame/sensor/realsense_d435_color/image", "/realsense/color/image_raw"),
            # ("/world/default/model/stretch/link/camera_color_optical_frame/sensor/realsense_d435_color/camera_info", "/realsense/color/camera_info"),
            # Realsense Depth
            ("/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/camera_info", "/realsense/depth/camera_info"),
            ("/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/points", "/realsense/depth/color/points"),
            ("/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/depth_image", "/realsense/depth/image_raw"),
            ("/world/default/model/stretch/link/camera_depth_optical_frame/sensor/realsense_d435/image", "realsense/depth/color/image_raw"),
            # Realsense IR 1
            # ("/world/default/model/stretch/link/camera_infra1_optical_frame/sensor/realsense_d435_ir/image", "realsense/infrared/image_raw"),
            # ("/world/default/model/stretch/link/camera_infra1_optical_frame/sensor/realsense_d435_ir/camera_info", "realsense/infrared/camera_info"),
            # Realsense IR 2
            # ("/world/default/model/stretch/link/camera_infra2_optical_frame/sensor/realsense_d435_ir2/image", "realsense/infrared2/image_raw"),
            # ("/world/default/model/stretch/link/camera_infra2_optical_frame/sensor/realsense_d435_ir2/camera_info", "realsense/infrared2/camera_info"),
        ],
        output='screen'
    )

    # Sensor Static TFs (until we can give frame_id argument in ignition sensor plugins)
    lidar_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='lidar_static_transform_publisher',
                        output='log',
                        arguments=['0', '0.0', '0.0', '0.0', '0.0', '0.0', 'laser', 'stretch/link_laser/gpu_lidar'])
    imu_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='imu_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'stretch/base_link/imu'])
    mag_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='mag_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'stretch/base_link/magnetometer'])
    wrist_imu_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='wrist_imu_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'link_wrist_yaw', 'stretch/link_wrist_yaw/wrist_imu'])
    realsense_imu_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='realsense_imu_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_gyro_frame', 'stretch/camera_gyro_frame/realsense_imu'])
    realsense_color_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='rgbd_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'camera_color_optical_frame', 'stretch/camera_color_optical_frame/realsense_d435_color'])
    realsense_depth_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='rgbd_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'camera_depth_optical_frame', 'stretch/camera_depth_optical_frame/realsense_d435'])
    realsense_ir_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='rgbd_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'camera_infra1_optical_frame', 'stretch/camera_infra1_optical_frame/realsense_d435_ir'])
    realsense_ir2_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='rgbd_static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'camera_infra2_optical_frame', 'stretch/camera_infra2_optical_frame/realsense_d435_ir2'])

    map_static_tf = Node(package='tf2_ros',
                        executable='static_transform_publisher',
                        name='static_transform_publisher',
                        output='log',
                        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'map'])

    # Controllers 
    # TODO (vatanaksoytezer): Use ros_ign_control when it is ready
    stretch_ignition_control_node = Node(
        package="stretch_ignition_control",
        executable="stretch_ignition_control_action_server",
        name="stretch_ignition_control",
        output="screen",
    )

    # Publish tf from odom
    odom2tf = Node(
        package="stretch_ignition",
        executable="odom2tf",
        name="odom2tf",
        output="screen",
    )

    # Publish static tfs
    stretch_sticky_static_tf_publisher = Node(
        package="stretch_ignition",
        executable="stretch_sticky_static_tf_publisher",
        name="stretch_sticky_static_tf_publisher",
        output="screen",
    )

    # Rotate Colored Images
    image_rotate = Node(
        package="image_rotate",
        executable="image_rotate",
        name="image_rotate",
        output="screen",
        remappings=[
            ('image', 'realsense/depth/color/image_raw'),
            ('camera_info', '/realsense/depth/camera_info'),
            ('rotated/image', 'realsense/depth/color/image_raw_rotated')
        ]
    )

    # Navigation
    navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_stretch_ignition, 'launch', 'navigation.launch.py')),
        condition=IfCondition(nav_arg),
    )

    return LaunchDescription(
        [
            # Launch Arguments
            DeclareLaunchArgument(
                'use_sim_time',
                default_value=use_sim_time,
                description="If true, use simulated clock"),
            DeclareLaunchArgument(
                'rviz',
                default_value=rviz_arg,
                description="If true, pop up an rviz instance"),
            DeclareLaunchArgument(
                'aws',
                default_value=aws_arg,
                description="If true, opens up an aws_robomaker world instead of an empty world"),
            DeclareLaunchArgument(
                'nav',
                default_value=nav_arg,
                description="If true, starts up navigation"),
            # Nodes and Launches
            empty_gazebo,
            aws_gazebo,
            spawn,
            bridge,
            robot_state_publisher,
            stretch_ignition_control_node,
            map_static_tf,
            odom2tf,
            stretch_sticky_static_tf_publisher,
            rviz,
            navigation
        ]
    )
