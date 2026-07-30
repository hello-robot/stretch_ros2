import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    dict_file_path = os.path.join(get_package_share_directory('stretch_core'), 'config', 'stretch_marker_dict.yaml')

    # Confirmed against a running D405 camera (ros2 topic list / camera_info).
    frame_id = 'gripper_camera_color_optical_frame'
    rgb_topic_name = '/gripper_camera/color/image_rect_raw'
    depth_topic_name = '/gripper_camera/aligned_depth_to_color/image_raw'
    camera_info_topic_name = '/gripper_camera/color/camera_info'

    detect_d405_aruco_markers = Node(
        name='detect_d405_aruco_node',
        package='stretch_core',
        executable='detect_aruco_markers',
        output='screen',
        parameters=[dict_file_path, {
            'frame_id': frame_id,
            'rgb_topic_name': rgb_topic_name,
            'depth_topic_name': depth_topic_name,
            'camera_info_topic_name': camera_info_topic_name,
        }],
        remappings=[
            ('/aruco/marker_array', '/gripper_camera/aruco/marker_array'),
            ('/aruco/axes', '/gripper_camera/aruco/axes'),
            ('/aruco/point_cloud2', '/gripper_camera/aruco/point_cloud2'),
            ('/aruco/wrist_top', '/gripper_camera/aruco/wrist_top'),
            ('/aruco/wrist_inside', '/gripper_camera/aruco/wrist_inside'),
        ],
        )

    return LaunchDescription([
        detect_d405_aruco_markers,
        ])
