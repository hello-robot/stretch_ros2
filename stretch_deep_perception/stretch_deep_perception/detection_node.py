#!/usr/bin/env python3

from unicodedata import name
import cv2
import ctypes
import rclpy
import sys

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
# from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import MarkerArray

import ros2_numpy
import message_filters

import struct

from . import detection_ros_markers as dr
from . import detection_2d_to_3d as d2


class DetectionNode:
    def __init__(self, detector, default_marker_name, node_name,
                 topic_base_name, fit_plane, min_box_side_m=None,
                 max_box_side_m=None, modify_3d_detections=None):
        self.rgb_image = None
        self.rgb_image_timestamp = None
        self.depth_image = None
        self.depth_image_timestamp = None        
        self.camera_info = None
        self.all_points = []
        self.publish_marker_point_clouds = True

        self.detector = detector
        
        self.marker_collection = dr.DetectionBoxMarkerCollection(default_marker_name)
        
        self.landmark_color_dict = self.detector.get_landmark_color_dict()
        self.topic_base_name = topic_base_name
        self.node_name = node_name
        self.fit_plane = fit_plane
        self.min_box_side_m = min_box_side_m
        self.max_box_side_m = max_box_side_m
        self.modify_3d_detections = modify_3d_detections
        self.image_count = 0
        
        
    def image_callback(self, ros_rgb_image, ros_depth_image, rgb_camera_info):
        self.rgb_image = ros2_numpy.numpify(ros_rgb_image)
        self.rgb_image_timestamp = ros_rgb_image.header.stamp
        self.depth_image = ros2_numpy.numpify(ros_depth_image)
        self.depth_image_timestamp = ros_depth_image.header.stamp
        self.camera_info = rgb_camera_info
        self.image_count = self.image_count + 1

        # OpenCV expects bgr images, but numpify by default returns rgb images.
        self.rgb_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)
        
        # TODO: Check if this operation can be handled by a ROS 2 method instead of
        # doing it manually
        ############
        time_diff_nanosec = abs(self.rgb_image_timestamp.nanosec - self.depth_image_timestamp.nanosec)
        time_diff_sec = abs(self.rgb_image_timestamp.sec - self.depth_image_timestamp.sec)
        time_diff = time_diff_sec + time_diff_nanosec*0.000001
        ############
        
        if time_diff > 0.0001:
            print('WARNING: The rgb image and the depth image were not taken at the same time.')
            print('         The time difference between their timestamps =', closest_time_diff, 's')

        # Rotate the image by 90deg to account for camera
        # orientation. In the future, this may be performed at the
        # image source.
        detection_box_image = cv2.rotate(self.rgb_image, cv2.ROTATE_90_CLOCKWISE)

        debug_input = False
        if debug_input: 
            print('DetectionNode.image_callback: received an image!')
            print('DetectionNode.image_callback: detection_box_image.shape =', detection_box_image.shape)
            cv2.imwrite('./output_images/deep_learning_input_' + str(self.image_count).zfill(4) + '.png', detection_box_image)
        
        debug_output = False
        detections_2d, output_image = self.detector.apply_to_image(detection_box_image, draw_output=debug_output)

        output_image = cv2.cvtColor(output_image, cv2.COLOR_BGR2RGB)

        if debug_output: 
            print('DetectionNode.image_callback: processed image with deep network!')
            print('DetectionNode.image_callback: output_image.shape =', output_image.shape)
            cv2.imwrite('./output_images/deep_learning_output_' + str(self.image_count).zfill(4) + '.png', output_image)

        if output_image is not None:
            output_image = ros2_numpy.msgify(Image, output_image, encoding='rgb8')
            if output_image is not None:
                self.visualize_object_detections_pub.publish(output_image)

        detections_3d = d2.detections_2d_to_3d(detections_2d, self.rgb_image, self.camera_info, self.depth_image, fit_plane=self.fit_plane, min_box_side_m=self.min_box_side_m, max_box_side_m=self.max_box_side_m)

        if self.modify_3d_detections is not None:
            detections_3d = self.modify_3d_detections(detections_3d)

        self.marker_collection.update(detections_3d, self.rgb_image_timestamp)
        
        marker_array = self.marker_collection.get_ros_marker_array(self.landmark_color_dict)
        include_axes = True
        include_z_axes = False
        axes_array = None
        axes_scale = 4.0
        if include_axes or include_z_axes: 
            axes_array = self.marker_collection.get_ros_axes_array(include_z_axes, include_axes, axes_scale=axes_scale)
        
        if self.publish_marker_point_clouds: 
            for marker in self.marker_collection:
                marker_points = marker.get_marker_point_cloud()
                self.add_point_array_to_point_cloud(marker_points)
                publish_plane_points = False
                if publish_plane_points: 
                    plane_points = marker.get_plane_fit_point_cloud()
                    self.add_point_array_to_point_cloud(plane_points)
            self.publish_point_cloud()
        self.visualize_markers_pub.publish(marker_array)
        if axes_array is not None: 
            self.visualize_axes_pub.publish(axes_array)
            

    def add_to_point_cloud(self, x_mat, y_mat, z_mat, mask):
        points = [[x, y, z] for x, y, z, m in zip(x_mat.flatten(), y_mat.flatten(), z_mat.flatten(), mask.flatten()) if m > 0]
        self.all_points.extend(points)

    def add_point_array_to_point_cloud(self, point_array):
        if point_array is not None: 
            self.all_points.extend(list(point_array))
            
    def publish_point_cloud(self):
        header = Header()
        header.frame_id = 'camera_color_optical_frame'
        header.stamp = self.node.get_clock().now().to_msg()
        fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='rgba', offset=12, datatype=PointField.UINT32, count=1)]
        r = 255
        g = 0
        b = 0
        a = 128
        rgba = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        points = [[x, y, z, rgba] for x, y, z in self.all_points]
        
        # TODO: The following code chunk is a substitute for the create_cloud method
        # in point_cloud2.py file in sensor_msgs in ROS 1; no equivalent in ROS 2
        # Check 'from sensor_msgs import point_cloud2' in ROS 1
        ###############
        is_bigendian = False
        field_names = None
        _DATATYPES = {}

        fmt = '>' if is_bigendian else '<'
        offset = 0
        for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
            if offset < field.offset:
                fmt += 'x' * (field.offset - offset)
                offset = field.offset
            if field.datatype not in _DATATYPES:
                print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
            else:
                datatype_fmt, datatype_length = _DATATYPES[field.datatype]
                fmt    += field.count * datatype_fmt
                offset += field.count * datatype_length
        cloud_struct = struct.Struct(fmt)

        buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        offset = 0

        # for p in points:
        #     pack_into(buff, offset, *p)
        #     offset += point_step

        point_cloud = PointCloud2(
                        header=header,
                        height=1,
                        width=len(points),
                        is_dense=False,
                        is_bigendian=False,
                        fields=fields,
                        point_step=cloud_struct.size,
                        row_step=cloud_struct.size * len(points),
                        data=buff.raw)
        ###############

        self.visualize_point_cloud_pub.publish(point_cloud)
        self.all_points = []
    
    def main(self):
        rclpy.init()
        self.node = rclpy.create_node(self.node_name)
        name = self.node.get_name()
        self.node.get_logger().info("{0} started".format(name))
        
        self.rgb_topic_name = '/camera/color/image_raw' #'/camera/infra1/image_rect_raw'
        self.rgb_image_subscriber = message_filters.Subscriber(self.node, Image, self.rgb_topic_name)

        self.depth_topic_name = '/camera/aligned_depth_to_color/image_raw'
        self.depth_image_subscriber = message_filters.Subscriber(self.node, Image, self.depth_topic_name)

        self.camera_info_subscriber = message_filters.Subscriber(self.node, CameraInfo, '/camera/color/camera_info')

        self.synchronizer = message_filters.TimeSynchronizer([self.rgb_image_subscriber, self.depth_image_subscriber, self.camera_info_subscriber], 10)
        self.synchronizer.registerCallback(self.image_callback)
        
        self.visualize_markers_pub = self.node.create_publisher(MarkerArray, '/' + self.topic_base_name + '/marker_array', 1)
        self.visualize_axes_pub = self.node.create_publisher(MarkerArray, '/' + self.topic_base_name + '/axes', 1)
        self.visualize_point_cloud_pub = self.node.create_publisher(PointCloud2, '/' + self.topic_base_name + '/point_cloud2', 1)

        self.visualize_object_detections_pub = self.node.create_publisher(Image, '/' + self.topic_base_name + '/color/image_with_bb', 1)

        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            print('interrupt received, so shutting down')

        self.node.destroy_node()
        rclpy.shutdown()
        
