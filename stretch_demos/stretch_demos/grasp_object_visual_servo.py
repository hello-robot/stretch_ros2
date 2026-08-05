#!/usr/bin/env python3

import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.logging
from rclpy.time import Time
import tf2_ros
import tf_transformations

from std_srvs.srv import Trigger
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

import hello_helpers.hello_misc as hm
import hello_helpers.fit_plane as fp
import stretch_funmap.manipulation_planning as mp

# Incremental build-out described in grasp-demo-status.md / the approved
# implementation plan.
#
# Increment 1: Stage 1 targeting only (reuses the existing FUNMAP
# scan-and-select pipeline unchanged, exactly as grasp_object.py does),
# converted to a real 3D point and logged.
#
# Increment 2 (this update): D405 subscriptions (fingertip ArUco markers +
# point cloud) and the Stage 2 local-tracking math, but LOG-ONLY -- no
# velocity commands are published yet. Verifies the tracking math is sane
# before any motion is wired in (Increment 3).

GRIPPER_CAMERA_FRAME_ID = 'gripper_camera_color_optical_frame'

# Marker-center -> true-fingertip-contact-point offsets, hand-measured and
# visually confirmed on hardware (see grasp-demo-status.md, Phase B).
# Expressed in each marker's own local (x_axis, y_axis, z_axis) frame, in
# meters.
FINGERTIP_OFFSETS = {
    'finger_left': (0.015, 0.008, -0.030),
    'finger_right': (-0.015, 0.008, -0.030),
}

# Stage 2 tracker tuning -- placeholders, not yet tuned on hardware.
STAGE2_SEARCH_RADIUS_MARGIN_M = 0.05  # added on top of Stage 1's width_m
STAGE2_MIN_SEARCH_RADIUS_M = 0.08
STAGE2_MIN_HEIGHT_ABOVE_SURFACE_M = 0.01  # see filter_points_above_local_surface()
STAGE2_FINGERTIP_EXCLUSION_RADIUS_M = 0.03
STAGE2_MIN_POINTS = 15
STAGE2_SMOOTHING_ALPHA = 0.3  # see smooth_estimate() -- placeholder, not yet tuned
STAGE2_TICK_PERIOD_S = 1.0 / 15.0
STAGE2_TEST_DURATION_S = 30.0  # Increment 2 only: auto-stop the log-only test loop


def smooth_estimate(previous_estimate_xyz, new_measurement_xyz, alpha):
    """Exponential moving average blend of a new per-tick position
    measurement with the previous smoothed estimate -- damps tick-to-tick
    jumps (e.g. from a textured/reflective object surface causing the set
    of valid depth points to shift between frames) without needing to know
    their exact cause. alpha in (0, 1]: higher trusts the new measurement
    more (more responsive, less smoothing); lower is smoother but adds lag.
    Pure function.
    """
    return alpha * new_measurement_xyz + (1.0 - alpha) * previous_estimate_xyz


def push_along_viewing_ray(point_xyz, push_distance_m):
    """Push a camera-frame point further from the camera's own origin
    (0,0,0) along its viewing ray by push_distance_m.

    The D405 only ever sees the near-facing surface of an object, so a
    plain centroid of visible points systematically undershoots the
    object's true center by roughly its radius, along the line of sight --
    same correction as `yolo_servo_perception.py`'s `grasp_center_xyz =
    center_xyz + (grasp_depth * center_ray)`. Since camera-frame points
    are already relative to the camera's own origin, the ray direction
    through any point is just that point's own unit direction vector, so
    no separate camera-pose lookup is needed. Pure function.
    """
    norm = np.linalg.norm(point_xyz)
    if norm < 1e-6:
        return point_xyz
    ray_direction = point_xyz / norm
    return point_xyz + push_distance_m * ray_direction


def transform_point(point_xyz, transform_stamped):
    """Apply a geometry_msgs/TransformStamped to a 3D point. Pure function
    (no ROS/self state) so it's testable with a synthetic TransformStamped.
    """
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    p = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0])
    return np.matmul(mat, p)[:3]


def get_marker_axes(orientation):
    """Derive a marker's local (x_axis, y_axis, z_axis) unit vectors from
    its pose orientation quaternion -- the inverse of how
    detect_aruco_markers.py's ArucoMarker built the quaternion from those
    same axes in the first place (tf_transformations.quaternion_from_matrix).
    Pure function.
    """
    mat = tf_transformations.quaternion_matrix(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    x_axis = mat[:3, 0]
    y_axis = mat[:3, 1]
    z_axis = mat[:3, 2]
    return x_axis, y_axis, z_axis


def get_fingertip_contact_point(marker, offset):
    """marker: a visualization_msgs/Marker for a fingertip ArUco marker.
    offset: (ox, oy, oz) tuple, see FINGERTIP_OFFSETS. Pure function.
    """
    position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
    x_axis, y_axis, z_axis = get_marker_axes(marker.pose.orientation)
    ox, oy, oz = offset
    return position + ox * x_axis + oy * y_axis + oz * z_axis


def filter_points_above_local_surface(points_xyz, min_height_above_surface_m):
    """Separate "the object" from "the support surface it's sitting on"
    within a local point-cloud crop, without any floor-relative reasoning.

    Fits a plane to points_xyz via SVD (hello_helpers.fit_plane.FitPlane --
    generic/reusable, not the room-scale, ground-referenced fit FUNMAP's
    find_closest_flat_surface() does) and keeps only the points whose
    unsigned distance from that fitted plane exceeds
    min_height_above_surface_m. Using the unsigned distance
    (FitPlane.abs_dist(), not the signed height()) deliberately sidesteps
    FitPlane's "towards_camera" orientation convention, which isn't known
    to match this D405 point cloud's frame -- we only care how far a point
    is from the fitted surface, not which side it's on.

    With a search region dominated by a flat surface (e.g. a tabletop)
    plus a much smaller object on it, the least-squares fit lands close to
    the surface (the majority of the points), so the object's points show
    up as clear outliers. Pure function.
    """
    if len(points_xyz) < 3:
        return points_xyz
    plane = fp.FitPlane()
    plane.fit_svd(points_xyz, verbose=False)
    dist = plane.abs_dist(points_xyz)
    return points_xyz[dist > min_height_above_surface_m]


def track_object_centroid(points_xyz, previous_estimate_xyz, search_radius_m,
                           min_height_above_surface_m=0.0,
                           exclude_points=None, exclude_radius_m=0.0, min_points=1):
    """Local point-cloud centroid tracker (Stage 2's core per-tick update).

    points_xyz: Nx3 numpy array, camera frame.
    previous_estimate_xyz: 3-vector, current tracked position, camera frame.
    min_height_above_surface_m: if > 0, apply
      filter_points_above_local_surface() after the spatial radius filter --
      needed in practice: a spatial-radius-only filter is dominated by
      whatever flat surface the object sits on (many more surface points
      than object points within any reasonably-sized search sphere), which
      pulls the centroid toward the surface rather than the object.
    exclude_points: optional list of 3-vectors (e.g. fingertip contact
      points) to exclude nearby points from (avoids the gripper's own
      fingers contaminating the object estimate).

    Returns (new_estimate_xyz_or_None, num_points_used). Pure function --
    no ROS/self state -- so it's unit-testable with synthetic arrays.
    """
    if points_xyz is None or len(points_xyz) == 0:
        return None, 0

    dist_to_estimate = np.linalg.norm(points_xyz - previous_estimate_xyz, axis=1)
    nearby = points_xyz[dist_to_estimate < search_radius_m]

    if min_height_above_surface_m > 0.0:
        nearby = filter_points_above_local_surface(nearby, min_height_above_surface_m)

    if exclude_points and len(nearby) > 0:
        mask = np.ones(len(nearby), dtype=bool)
        for ex in exclude_points:
            mask &= (np.linalg.norm(nearby - ex, axis=1) > exclude_radius_m)
        nearby = nearby[mask]

    if len(nearby) < min_points:
        return None, len(nearby)

    return nearby.mean(axis=0), len(nearby)


class GraspObjectVisualServoNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.debug_directory = None
        self.manipulation_view = None

        self.fingertip_lock = threading.Lock()
        self.latest_fingertip_marker_array = None

        self.gripper_point_cloud_lock = threading.Lock()
        self.gripper_point_cloud = None

        self.stage2_active = False
        self.stage2_start_time_s = None
        self.tracked_object_xyz_camera = None
        self.stage2_search_radius_m = STAGE2_MIN_SEARCH_RADIUS_M
        self.object_width_m = None

    # DATA CALLBACKS (D405) ############

    def fingertip_marker_array_callback(self, marker_array):
        with self.fingertip_lock:
            self.latest_fingertip_marker_array = marker_array

    def gripper_point_cloud_callback(self, point_cloud_msg):
        with self.gripper_point_cloud_lock:
            self.gripper_point_cloud = point_cloud_msg

    def get_fingertip_contact_points(self):
        """Returns a dict {'finger_left': xyz_or_None, 'finger_right': xyz_or_None}
        from the most recently received D405 ArUco marker array, in camera frame.
        """
        with self.fingertip_lock:
            marker_array = self.latest_fingertip_marker_array
        contact_points = {'finger_left': None, 'finger_right': None}
        if marker_array is None:
            return contact_points
        for marker in marker_array.markers:
            if marker.text in FINGERTIP_OFFSETS:
                contact_points[marker.text] = get_fingertip_contact_point(
                    marker, FINGERTIP_OFFSETS[marker.text])
        return contact_points

    # STAGE 1 -> STAGE 2 HANDOFF ############

    def find_grasp_target(self):
        """Stage 1: one-shot FUNMAP scan + surface/largest-blob object
        selection (stretch_funmap.manipulation_planning.ManipulationView,
        unmodified). Returns (grasp_target, object_xyz_base_link), or
        (None, None) if no object was found.
        """
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory, self.get_tool())
        manip = self.manipulation_view
        manip.move_head(self.move_to_pose)

        scan_time_s = 4.0
        start_time_s = time.time()
        while (time.time() - start_time_s) < scan_time_s:
            manip.update(self.point_cloud, self.tf2_buffer)

        grasp_target = manip.get_grasp_target(self.tf2_buffer)
        if grasp_target is None:
            return None, None

        h = manip.max_height_im
        xyz_pix = [grasp_target['location_xy_pix'][0],
                   grasp_target['location_xy_pix'][1],
                   grasp_target['location_z_pix']]
        object_xyz_base_link = h.get_pix_in_frame(xyz_pix, 'base_link', self.tf2_buffer)

        return grasp_target, object_xyz_base_link

    def start_stage2_tracking(self, object_xyz_base_link, width_m):
        """One-time handoff: convert Stage 1's object position (base_link)
        into the D405 camera frame, seed the Stage 2 tracker, and activate
        the tracking timer. Returns True on success.
        """
        try:
            transform = self.tf2_buffer.lookup_transform(
                GRIPPER_CAMERA_FRAME_ID, 'base_link', Time())
        except tf2_ros.TransformException as e:
            self.logger.error('Stage 1->2 handoff failed: could not look up transform from base_link to {0}: {1}'.format(
                GRIPPER_CAMERA_FRAME_ID, e))
            return False

        self.tracked_object_xyz_camera = transform_point(object_xyz_base_link, transform)
        self.stage2_search_radius_m = max(STAGE2_MIN_SEARCH_RADIUS_M, width_m + STAGE2_SEARCH_RADIUS_MARGIN_M)
        self.object_width_m = width_m
        self.stage2_start_time_s = time.time()
        self.stage2_active = True
        self.logger.info(
            'Stage 2 handoff: seeded tracker at camera-frame xyz=({0:.3f}, {1:.3f}, {2:.3f}) m, search_radius={3:.3f} m'.format(
                self.tracked_object_xyz_camera[0], self.tracked_object_xyz_camera[1],
                self.tracked_object_xyz_camera[2], self.stage2_search_radius_m))
        return True

    # STAGE 2 PER-TICK TRACKING (log-only for Increment 2) ############

    def stage2_tracking_tick(self):
        if not self.stage2_active:
            return

        if (time.time() - self.stage2_start_time_s) > STAGE2_TEST_DURATION_S:
            self.logger.info('Increment 2 test window finished ({0:.0f}s elapsed); stopping tracking loop.'.format(
                STAGE2_TEST_DURATION_S))
            self.stage2_active = False
            return

        with self.gripper_point_cloud_lock:
            cloud_msg = self.gripper_point_cloud

        if cloud_msg is None:
            self.logger.warning('Stage 2 tick: no D405 point cloud received yet.')
            return

        points_xyz = point_cloud2.read_points_numpy(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)

        fingertips = self.get_fingertip_contact_points()
        exclude_points = [p for p in fingertips.values() if p is not None]

        was_active_before_tick = self.tracked_object_xyz_camera is not None
        new_estimate, num_points = track_object_centroid(
            points_xyz, self.tracked_object_xyz_camera, self.stage2_search_radius_m,
            min_height_above_surface_m=STAGE2_MIN_HEIGHT_ABOVE_SURFACE_M,
            exclude_points=exclude_points, exclude_radius_m=STAGE2_FINGERTIP_EXCLUSION_RADIUS_M,
            min_points=STAGE2_MIN_POINTS)

        if new_estimate is None:
            if was_active_before_tick and (time.time() - self.stage2_start_time_s) < (2.0 * STAGE2_TICK_PERIOD_S):
                # First tick(s) right after the Stage 1->2 handoff finding
                # ~0 points likely means Stage 1's result was already
                # stale (object moved between the scan and the D405
                # getting a lock), not a tracking failure -- log it
                # distinctly rather than folding it into a generic
                # "lost" message.
                self.logger.warning(
                    'Stage 2 tick: only {0} points found immediately after Stage 1->2 handoff -- '
                    'Stage 1 result may be stale.'.format(num_points))
            else:
                self.logger.warning('Stage 2 tick: target lost ({0} points in search region).'.format(num_points))
            return

        # Blend with the previous estimate rather than replacing it
        # outright -- damps tick-to-tick jumps (e.g. a textured object
        # surface causing which points have valid depth to shift between
        # frames) before this position ever becomes a velocity command
        # (Increment 3+). self.tracked_object_xyz_camera is always a real
        # value here (seeded by start_stage2_tracking() before Stage 2
        # ever activates), so there's always a previous estimate to blend
        # with, including on the very first tick (blended with the Stage
        # 1->2 handoff position).
        smoothed_estimate = smooth_estimate(
            self.tracked_object_xyz_camera, new_estimate, STAGE2_SMOOTHING_ALPHA)
        # Internal tracking state stays on the raw (uncorrected) smoothed
        # position -- next tick's search radius must stay centered on
        # where the real, visible points actually are, not on empty space
        # past the object's near surface.
        self.tracked_object_xyz_camera = smoothed_estimate

        # The reported/consumed position, on the other hand, gets pushed
        # out along the viewing ray to correct for only ever seeing the
        # object's near-facing surface (see push_along_viewing_ray()).
        reported_position = push_along_viewing_ray(smoothed_estimate, self.object_width_m / 2.0)

        if fingertips['finger_left'] is not None and fingertips['finger_right'] is not None:
            between_fingertips = (fingertips['finger_left'] + fingertips['finger_right']) / 2.0
            position_error = reported_position - between_fingertips
            self.logger.info(
                'Stage 2 tick: object=({0:.3f},{1:.3f},{2:.3f}) between_fingertips=({3:.3f},{4:.3f},{5:.3f}) '
                'error=({6:.3f},{7:.3f},{8:.3f}) |error|={9:.3f} m [{10} pts]'.format(
                    reported_position[0], reported_position[1], reported_position[2],
                    between_fingertips[0], between_fingertips[1], between_fingertips[2],
                    position_error[0], position_error[1], position_error[2],
                    float(np.linalg.norm(position_error)), num_points))
        else:
            self.logger.info(
                'Stage 2 tick: object=({0:.3f},{1:.3f},{2:.3f}) [{3} pts] (fingertips not both visible)'.format(
                    reported_position[0], reported_position[1], reported_position[2], num_points))

        self.publish_debug_marker(reported_position)

    def publish_debug_marker(self, xyz_camera_frame):
        marker = Marker()
        marker.header.frame_id = GRIPPER_CAMERA_FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'grasp_object_visual_servo'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(xyz_camera_frame[0])
        marker.pose.position.y = float(xyz_camera_frame[1])
        marker.pose.position.z = float(xyz_camera_frame[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.debug_marker_pub.publish(marker)

    # SERVICE CALLBACK ############

    def trigger_grasp_object_visual_servo_callback(self, request, response):
        self.logger.info('Stow the arm.')
        self.stow_the_robot()

        self.logger.info('Stage 1: scanning for a grasp target.')
        grasp_target, object_xyz_base_link = self.find_grasp_target()
        if grasp_target is None:
            return Trigger.Response(
                success=False,
                message='Stage 1 failed to find a grasp target'
            )

        self.logger.info(
            'Stage 1 object location (base_link): x={0:.3f}, y={1:.3f}, z={2:.3f} m'.format(
                object_xyz_base_link[0], object_xyz_base_link[1], object_xyz_base_link[2]))
        self.logger.info('Stage 1 object width_m = {0:.3f} m'.format(grasp_target['width_m']))

        if not self.start_stage2_tracking(object_xyz_base_link, grasp_target['width_m']):
            return Trigger.Response(
                success=False,
                message='Stage 1 found a target, but the Stage 1->2 handoff failed (see log).'
            )

        return Trigger.Response(
            success=True,
            message='Stage 1 found a grasp target; Stage 2 log-only tracking active for {0:.0f}s '
                    '(Increment 2: no motion yet).'.format(STAGE2_TEST_DURATION_S)
        )

    def main(self):
        hm.HelloNode.main(self, 'grasp_object_visual_servo', 'grasp_object_visual_servo', wait_for_first_pointcloud=False)
        self.logger = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('debug_directory', '')
        self.debug_directory = self.get_parameter('debug_directory').value or None
        self.logger.info('Using the following directory for debugging files: {0}'.format(self.debug_directory))

        self.declare_parameter('dryrun', False)
        self.dryrun = self.get_parameter('dryrun').value

        self.gripper_aruco_subscriber = self.create_subscription(
            MarkerArray, '/gripper_camera/aruco/marker_array', self.fingertip_marker_array_callback,
            qos_profile=1, callback_group=self.callback_group)

        self.gripper_point_cloud_subscriber = self.create_subscription(
            PointCloud2, '/gripper_camera/depth/color/points', self.gripper_point_cloud_callback,
            qos_profile=1, callback_group=self.callback_group)

        self.debug_marker_pub = self.create_publisher(Marker, '/grasp_object_visual_servo/debug/tracked_object', 1)

        self.trigger_grasp_object_visual_servo_service = self.create_service(
            Trigger,
            '/grasp_object_visual_servo/trigger_grasp_object_visual_servo',
            callback=self.trigger_grasp_object_visual_servo_callback,
            callback_group=self.callback_group)

        self.stage2_timer = self.create_timer(
            STAGE2_TICK_PERIOD_S, self.stage2_tracking_tick, callback_group=self.callback_group)

        self.logger.info('Grasp object visual servo node is ready! (Increment 2: Stage 1 + log-only Stage 2 tracking)')


def main():
    try:
        node = GraspObjectVisualServoNode()
        node.main()
        node.new_thread.join()
    except KeyboardInterrupt:
        rclpy.logging.get_logger('grasp_object_visual_servo').info('interrupt received, so shutting down')


if __name__ == '__main__':
    main()
