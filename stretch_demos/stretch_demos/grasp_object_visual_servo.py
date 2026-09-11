#!/usr/bin/env python3

import threading
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy.logging
from rclpy.parameter import Parameter
from rclpy.time import Time
import tf2_ros
import tf_transformations

from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import hello_helpers.hello_misc as hm
import hello_helpers.fit_plane as fp
import stretch_funmap.manipulation_planning as mp
import stretch_funmap.navigate as nv

# Stage 1: FUNMAP head scan + object selection. Stage 2: D405 point-cloud tracking,
# anchored to Stage 1's result, currently log-only. See grasp-demo-status.md.

GRIPPER_CAMERA_FRAME_ID = 'gripper_camera_color_optical_frame'

# Own head-scan pose, not ManipulationView.move_head() (shared with other demos).
HEAD_SCAN_PAN_RAD = -np.pi / 2.0 + 0.1  # flip the +0.1 sign if it moves the wrong way
HEAD_SCAN_TILT_RAD = -0.8
HEAD_SCAN_SETTLE_TIME_S = 0.5

# Hand-measured marker -> fingertip-contact-point offsets, marker-local axes, meters.
FINGERTIP_OFFSETS = {
    'finger_left': (0.015, 0.008, -0.030),
    'finger_right': (-0.015, 0.008, -0.030),
}

# Stage 2 tuning, not yet tuned on hardware.
STAGE2_SEARCH_RADIUS_MARGIN_M = 0.05
STAGE2_MIN_SEARCH_RADIUS_M = 0.08
STAGE2_MIN_HEIGHT_ABOVE_SURFACE_M = 0.01
STAGE2_CONNECTIVITY_RADIUS_M = 0.01
STAGE2_CLUSTER_EXTENT_MARGIN_M = 0.03
STAGE2_FINGERTIP_EXCLUSION_RADIUS_M = 0.03
STAGE2_MIN_POINTS = 15
STAGE2_SMOOTHING_ALPHA = 0.3
STAGE2_MAX_DRIFT_FROM_ANCHOR_M = 0.08
STAGE2_TICK_PERIOD_S = 1.0 / 15.0
STAGE2_TEST_DURATION_S = 30.0  # Increment 2 only: auto-stop the log-only test loop
STAGE2_LOG_THROTTLE_S = 1.0  # tracking still runs at full rate; only the log is throttled

# Pregrasp tuning. Can't reach an object offset sideways by more than the arm's
# reach -- driving forward/backward alone can't fix that (see status doc).
PREGRASP_MAX_DRIVE_FORWARD_M = 1.0  # defensive clamp on the alignment drive
PREGRASP_LIFT_OFFSET_FROM_OBJECT_M = 0.20  # added to object height; negative = below
PREGRASP_ARM_EXTENSION_STANDOFF_M = 0.45  # stop this far short of the object
PREGRASP_FIXED_WRIST_YAW_RAD = -0.0786  # hand-calibrated "straight" for this robot
DEX_WRIST_TOOL_NAMES = {'tool_stretch_dex_wrist', 'eoa_wrist_dw3_tool_sg3', 'eoa_wrist_dw3_tool_sg3_pro'}
PREGRASP_FIXED_WRIST_PITCH_RAD = -0.6  # negative = tilted down
PREGRASP_FIXED_WRIST_ROLL_RAD = 0.0798  # hand-calibrated "straight" for this robot
PREGRASP_DRIVE_VELOCITY_MPS = 0.1  # overrides MoveBase's default (0.2), this node only
PREGRASP_GRIPPER_OPEN_APERTURE_M = 0.30  # above the true max (~0.24m); driver clamps it
PREGRASP_DRIVE_SETTLE_TIME_S = 0.5  # let odom/TF catch up after the drive reports done


def smooth_estimate(previous_estimate_xyz, new_measurement_xyz, alpha):
    """Exponential moving average. Higher alpha trusts the new measurement more."""
    return alpha * new_measurement_xyz + (1.0 - alpha) * previous_estimate_xyz


def push_along_viewing_ray(point_xyz, push_distance_m):
    """Push a point along its own viewing ray -- corrects for only seeing an
    object's near-facing surface."""
    norm = np.linalg.norm(point_xyz)
    if norm < 1e-6:
        return point_xyz
    ray_direction = point_xyz / norm
    return point_xyz + push_distance_m * ray_direction


def transform_point(point_xyz, transform_stamped):
    """Apply a geometry_msgs/TransformStamped to a 3D point."""
    t = transform_stamped.transform.translation
    q = transform_stamped.transform.rotation
    mat = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[0, 3] = t.x
    mat[1, 3] = t.y
    mat[2, 3] = t.z
    p = np.array([point_xyz[0], point_xyz[1], point_xyz[2], 1.0])
    return np.matmul(mat, p)[:3]


def get_marker_axes(orientation):
    """Local (x_axis, y_axis, z_axis) unit vectors of a marker from its orientation."""
    mat = tf_transformations.quaternion_matrix(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    return mat[:3, 0], mat[:3, 1], mat[:3, 2]


def get_fingertip_contact_point(marker, offset):
    """offset: (ox, oy, oz), see FINGERTIP_OFFSETS."""
    position = np.array([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
    x_axis, y_axis, z_axis = get_marker_axes(marker.pose.orientation)
    ox, oy, oz = offset
    return position + ox * x_axis + oy * y_axis + oz * z_axis


def filter_points_above_local_surface(points_xyz, min_height_above_surface_m):
    """Keeps points farther than min_height_above_surface_m from a fitted (SVD)
    plane -- separates an object from the surface it's on."""
    if len(points_xyz) < 3:
        return points_xyz
    plane = fp.FitPlane()
    plane.fit_svd(points_xyz, verbose=False)
    dist = plane.abs_dist(points_xyz)
    return points_xyz[dist > min_height_above_surface_m]


def connected_cluster_near_point(points_xyz, seed_point, connectivity_radius_m, max_extent_m=None):
    """Flood-fill cluster from the point nearest seed_point, joining points within
    connectivity_radius_m -- rejects a disjoint cluster even if it has more points.
    max_extent_m, if given, caps growth to that distance from seed_point itself."""
    if len(points_xyz) == 0:
        return points_xyz

    dists_to_seed = np.linalg.norm(points_xyz - seed_point, axis=1)
    start_idx = int(np.argmin(dists_to_seed))

    if max_extent_m is not None:
        eligible_mask = dists_to_seed <= max_extent_m
    else:
        eligible_mask = np.ones(len(points_xyz), dtype=bool)

    in_cluster = np.zeros(len(points_xyz), dtype=bool)
    in_cluster[start_idx] = True
    remaining_mask = eligible_mask.copy()
    remaining_mask[start_idx] = False
    frontier_indices = np.array([start_idx])

    while len(frontier_indices) > 0:
        remaining_indices = np.nonzero(remaining_mask)[0]
        if len(remaining_indices) == 0:
            break

        frontier_points = points_xyz[frontier_indices]
        remaining_points = points_xyz[remaining_indices]
        dists = np.linalg.norm(
            remaining_points[:, None, :] - frontier_points[None, :, :], axis=2)
        newly_added_mask = np.any(dists < connectivity_radius_m, axis=1)
        newly_added_indices = remaining_indices[newly_added_mask]

        if len(newly_added_indices) == 0:
            break

        in_cluster[newly_added_indices] = True
        remaining_mask[newly_added_indices] = False
        frontier_indices = newly_added_indices

    return points_xyz[in_cluster]


def track_object_centroid(points_xyz, previous_estimate_xyz, search_radius_m,
                           min_height_above_surface_m=0.0, connectivity_radius_m=0.0,
                           cluster_max_extent_m=None,
                           exclude_points=None, exclude_radius_m=0.0, min_points=1):
    """Stage 2's per-tick centroid update, camera frame: radius filter, then
    optional height, connected-component, and fingertip-exclusion filters.
    Returns (new_estimate_or_None, num_points_used, surviving_points,
    points_after_radius_filter); the latter two are for debug visualization."""
    if points_xyz is None or len(points_xyz) == 0:
        return None, 0, np.empty((0, 3)), np.empty((0, 3))

    dist_to_estimate = np.linalg.norm(points_xyz - previous_estimate_xyz, axis=1)
    nearby = points_xyz[dist_to_estimate < search_radius_m]
    points_after_radius_filter = nearby

    if min_height_above_surface_m > 0.0:
        nearby = filter_points_above_local_surface(nearby, min_height_above_surface_m)

    if connectivity_radius_m > 0.0 and len(nearby) > 0:
        nearby = connected_cluster_near_point(
            nearby, previous_estimate_xyz, connectivity_radius_m, max_extent_m=cluster_max_extent_m)

    if exclude_points and len(nearby) > 0:
        mask = np.ones(len(nearby), dtype=bool)
        for ex in exclude_points:
            mask &= (np.linalg.norm(nearby - ex, axis=1) > exclude_radius_m)
        nearby = nearby[mask]

    if len(nearby) < min_points:
        return None, len(nearby), nearby, points_after_radius_filter

    return nearby.mean(axis=0), len(nearby), nearby, points_after_radius_filter


class GraspObjectVisualServoNode(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        self.debug_directory = None
        self.manipulation_view = None
        self.move_base = None
        self.tool = None

        self.joint_states_lock = threading.Lock()
        self.joint_states = None
        self.wrist_position = None
        self.lift_position = None
        self.left_finger_position = None

        self.fingertip_lock = threading.Lock()
        self.latest_fingertip_marker_array = None

        self.gripper_point_cloud_lock = threading.Lock()
        self.gripper_point_cloud = None

        self.stage2_active = False
        self.stage2_start_time_s = None
        self.tracked_object_xyz_camera = None
        self.stage2_search_radius_m = STAGE2_MIN_SEARCH_RADIUS_M
        self.object_width_m = None
        self.anchor_xyz_base_link = None

    def joint_states_callback(self, joint_states):
        with self.joint_states_lock:
            self.joint_states = joint_states
        wrist_position, wrist_velocity, wrist_effort = hm.get_wrist_state(joint_states)
        self.wrist_position = wrist_position
        lift_position, lift_velocity, lift_effort = hm.get_lift_state(joint_states)
        self.lift_position = lift_position
        self.left_finger_position, temp1, temp2 = hm.get_left_finger_state(joint_states)

    # DATA CALLBACKS (D405) ############

    def fingertip_marker_array_callback(self, marker_array):
        with self.fingertip_lock:
            self.latest_fingertip_marker_array = marker_array

    def gripper_point_cloud_callback(self, point_cloud_msg):
        with self.gripper_point_cloud_lock:
            self.gripper_point_cloud = point_cloud_msg

    def get_fingertip_contact_points(self):
        """{'finger_left': xyz_or_None, 'finger_right': xyz_or_None}, camera frame."""
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
        """Stage 1: one-shot FUNMAP scan + object selection. Returns (grasp_target,
        object_xyz_base_link), or (None, None) if no object was found."""
        self.manipulation_view = mp.ManipulationView(self.tf2_buffer, self.debug_directory, self.get_tool())
        manip = self.manipulation_view
        self.move_to_pose({'joint_head_pan': HEAD_SCAN_PAN_RAD, 'joint_head_tilt': HEAD_SCAN_TILT_RAD})
        time.sleep(HEAD_SCAN_SETTLE_TIME_S)

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

    def drive(self, forward_m):
        if self.dryrun:
            return
        tolerance_distance_m = 0.005
        if forward_m > 0:
            at_goal = self.move_base.forward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        else:
            at_goal = self.move_base.backward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        self.logger.warning('Pregrasp alignment [layers 1+2]: MoveBase reports at_goal={0} for the {1:.4f} m drive.'.format(
            at_goal, forward_m))

    def move_to_simple_pregrasp_pose(self, grasp_target, object_xyz_base_link):
        """Non-rotating pregrasp move: drive to align x, lift, extend, then fixed
        wrist yaw/pitch/roll. Skipped entirely under dryrun. Returns (success,
        object_xyz_base_link_now) -- callers must use this returned position, not the
        one passed in, for the Stage 1->2 handoff: only the drive changes base_link's
        pose, and this re-derives the object's position right after it."""
        if self.dryrun:
            self.logger.info('dryrun: skipping the simple pregrasp move.')
            return True, object_xyz_base_link

        max_lift_m = 1.09
        min_extension_m = 0.01
        max_extension_m = 0.5

        if self.lift_position is None or self.wrist_position is None:
            self.logger.error('move_to_simple_pregrasp_pose: lift/wrist position unavailable.')
            return False, object_xyz_base_link

        self.publish_pregrasp_alignment_marker('before', object_xyz_base_link)
        self.logger.warning(
            'Pregrasp alignment [layer 3]: x distance base_link->object BEFORE drive = {0:.3f} m (target: 0.000 m)'.format(
                object_xyz_base_link[0]))
        forward_m = object_xyz_base_link[0]
        forward_m = max(min(forward_m, PREGRASP_MAX_DRIVE_FORWARD_M), -PREGRASP_MAX_DRIVE_FORWARD_M)
        self.logger.info('Pregrasp (simple): driving {0:.3f} m to align x.'.format(forward_m))
        self.drive(forward_m)
        time.sleep(PREGRASP_DRIVE_SETTLE_TIME_S)

        # Re-derive after the drive (live TF, not cached).
        h = self.manipulation_view.max_height_im
        xyz_pix = [grasp_target['location_xy_pix'][0],
                   grasp_target['location_xy_pix'][1],
                   grasp_target['location_z_pix']]
        object_xyz_base_link_now = h.get_pix_in_frame(xyz_pix, 'base_link', self.tf2_buffer)
        self.logger.info(
            'Pregrasp (simple): after driving, object now at x={0:.3f}, y={1:.3f}, z={2:.3f} m (base_link).'.format(
                object_xyz_base_link_now[0], object_xyz_base_link_now[1], object_xyz_base_link_now[2]))
        self.logger.warning(
            'Pregrasp alignment [layer 3]: x distance base_link->object AFTER drive = {0:.3f} m (target: 0.000 m)'.format(
                object_xyz_base_link_now[0]))
        self.publish_pregrasp_alignment_marker('after', object_xyz_base_link_now)

        target_lift_m = object_xyz_base_link_now[2] + PREGRASP_LIFT_OFFSET_FROM_OBJECT_M
        target_lift_m = max(min(target_lift_m, max_lift_m), 0.1)
        self.logger.info('Pregrasp (simple): moving lift to {0:.3f} m.'.format(target_lift_m))
        self.move_to_pose({'joint_lift': target_lift_m})

        extension_m = abs(object_xyz_base_link_now[1]) - PREGRASP_ARM_EXTENSION_STANDOFF_M
        extension_m = max(min(extension_m, max_extension_m), min_extension_m)
        self.logger.info('Pregrasp (simple): extending wrist to {0:.3f} m.'.format(extension_m))
        self.move_to_pose({'wrist_extension': extension_m})

        self.logger.info('Pregrasp (simple): rotating wrist yaw to fixed {0:.3f} rad.'.format(
            PREGRASP_FIXED_WRIST_YAW_RAD))
        self.move_to_pose({'joint_wrist_yaw': PREGRASP_FIXED_WRIST_YAW_RAD})

        # Dex wrists: pitch/roll aren't otherwise touched, so set them explicitly.
        if self.tool in DEX_WRIST_TOOL_NAMES:
            self.logger.info(
                'Pregrasp (simple): setting wrist pitch/roll to fixed {0:.3f}/{1:.3f} rad (tool={2}).'.format(
                    PREGRASP_FIXED_WRIST_PITCH_RAD, PREGRASP_FIXED_WRIST_ROLL_RAD, self.tool))
            self.move_to_pose({'joint_wrist_pitch': PREGRASP_FIXED_WRIST_PITCH_RAD,
                                'joint_wrist_roll': PREGRASP_FIXED_WRIST_ROLL_RAD})

        self.logger.info('Open the gripper.')
        self.move_to_pose({'gripper_aperture': PREGRASP_GRIPPER_OPEN_APERTURE_M})

        return True, object_xyz_base_link_now

    def start_stage2_tracking(self, object_xyz_base_link, width_m):
        """One-time handoff: seed the Stage 2 tracker and activate the tracking
        timer. Returns True on success."""
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
        # Re-projected into camera frame fresh each tick; see stage2_tracking_tick().
        self.anchor_xyz_base_link = np.array(object_xyz_base_link)
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

        # Anchor-centered, not self-referential -- avoids locking onto a
        # nearby-but-wrong cluster and never escaping it.
        was_active_before_tick = self.tracked_object_xyz_camera is not None
        try:
            anchor_transform = self.tf2_buffer.lookup_transform(
                GRIPPER_CAMERA_FRAME_ID, 'base_link', Time())
            anchor_xyz_camera_now = transform_point(self.anchor_xyz_base_link, anchor_transform)
        except tf2_ros.TransformException as e:
            self.logger.warning('Stage 2 tick: could not look up the anchor position (TF lookup failed): {0}'.format(e))
            return

        cluster_max_extent_m = (self.object_width_m / 2.0) + STAGE2_CLUSTER_EXTENT_MARGIN_M
        new_estimate, num_points, surviving_points, points_after_radius_filter = track_object_centroid(
            points_xyz, anchor_xyz_camera_now, self.stage2_search_radius_m,
            min_height_above_surface_m=STAGE2_MIN_HEIGHT_ABOVE_SURFACE_M,
            connectivity_radius_m=STAGE2_CONNECTIVITY_RADIUS_M,
            cluster_max_extent_m=cluster_max_extent_m,
            exclude_points=exclude_points, exclude_radius_m=STAGE2_FINGERTIP_EXCLUSION_RADIUS_M,
            min_points=STAGE2_MIN_POINTS)

        self.publish_debug_points(surviving_points)
        self.publish_debug_points(points_after_radius_filter, self.debug_points_pre_height_pub, rgb=(0.0, 1.0, 0.0))

        if new_estimate is None:
            if was_active_before_tick and (time.time() - self.stage2_start_time_s) < (2.0 * STAGE2_TICK_PERIOD_S):
                # Right after handoff, likely a stale Stage 1 result, not a tracking failure.
                self.logger.warning(
                    'Stage 2 tick: only {0} points found immediately after Stage 1->2 handoff -- '
                    'Stage 1 result may be stale.'.format(num_points))
            else:
                self.logger.warning('Stage 2 tick: target lost ({0} points in search region).'.format(num_points))
            return

        # Damps tick-to-tick jumps before this feeds a velocity command (Increment 3+).
        smoothed_estimate = smooth_estimate(
            self.tracked_object_xyz_camera, new_estimate, STAGE2_SMOOTHING_ALPHA)

        # Safety net; should rarely trigger given the anchor-centered search above.
        drift_from_anchor_m = float(np.linalg.norm(smoothed_estimate - anchor_xyz_camera_now))
        if drift_from_anchor_m > STAGE2_MAX_DRIFT_FROM_ANCHOR_M:
            self.logger.warning(
                'Stage 2 tick: smoothed estimate drifted {0:.3f} m from the Stage 1 anchor '
                '(max {1:.3f} m) -- resetting to the anchor.'.format(
                    drift_from_anchor_m, STAGE2_MAX_DRIFT_FROM_ANCHOR_M))
            smoothed_estimate = anchor_xyz_camera_now

        self.tracked_object_xyz_camera = smoothed_estimate

        # Pushed along the viewing ray: we only ever see the object's near-facing surface.
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
                    float(np.linalg.norm(position_error)), num_points),
                throttle_duration_sec=STAGE2_LOG_THROTTLE_S)
        else:
            self.logger.info(
                'Stage 2 tick: object=({0:.3f},{1:.3f},{2:.3f}) [{3} pts] (fingertips not both visible)'.format(
                    reported_position[0], reported_position[1], reported_position[2], num_points),
                throttle_duration_sec=STAGE2_LOG_THROTTLE_S)

        self.publish_debug_marker(self.debug_marker_raw_pub, smoothed_estimate, (0.0, 0.6, 1.0))  # cyan: raw
        self.publish_debug_marker(self.debug_marker_pub, reported_position, (1.0, 0.5, 0.0))  # orange: pushed

    def publish_debug_marker(self, publisher, xyz_camera_frame, rgb):
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
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.8
        publisher.publish(marker)

    def publish_debug_points(self, points_xyz, publisher=None, rgb=(1.0, 0.0, 1.0)):
        publisher = publisher if publisher is not None else self.debug_points_pub
        marker = Marker()
        marker.header.frame_id = GRIPPER_CAMERA_FRAME_ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'grasp_object_visual_servo'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.006
        marker.scale.y = 0.006
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.9
        marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in points_xyz]
        publisher.publish(marker)

    def publish_pregrasp_alignment_marker(self, label, object_xyz_base_link):
        """Sphere + text at the object's base_link position, green->red as |x|
        grows. label is 'before' or 'after'; both stay visible at once."""
        x = object_xyz_base_link[0]
        good_threshold_m = 0.02
        bad_threshold_m = 0.15
        t = max(0.0, min(1.0, (abs(x) - good_threshold_m) / (bad_threshold_m - good_threshold_m)))
        rgb = (t, 1.0 - t, 0.0)  # green (aligned) -> red (far off)
        id_offset = 0 if label == 'before' else 2

        sphere = Marker()
        sphere.header.frame_id = 'base_link'
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = 'grasp_object_visual_servo_pregrasp_alignment'
        sphere.id = id_offset
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(object_xyz_base_link[0])
        sphere.pose.position.y = float(object_xyz_base_link[1])
        sphere.pose.position.z = float(object_xyz_base_link[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.04
        sphere.scale.y = 0.04
        sphere.scale.z = 0.04
        sphere.color.r = rgb[0]
        sphere.color.g = rgb[1]
        sphere.color.b = rgb[2]
        sphere.color.a = 0.9
        self.debug_pregrasp_alignment_pub.publish(sphere)

        text = Marker()
        text.header.frame_id = 'base_link'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'grasp_object_visual_servo_pregrasp_alignment'
        text.id = id_offset + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        # Keeps the two labels apart even when both spheres land close together.
        text_z_offset_m = 0.12 if label == 'before' else -0.08
        text.pose.position.x = float(object_xyz_base_link[0])
        text.pose.position.y = float(object_xyz_base_link[1])
        text.pose.position.z = float(object_xyz_base_link[2]) + text_z_offset_m
        text.pose.orientation.w = 1.0
        text.scale.z = 0.05
        text.color.r = rgb[0]
        text.color.g = rgb[1]
        text.color.b = rgb[2]
        text.color.a = 1.0
        text.text = '{0}: x={1:.3f} m'.format(label, x)
        self.debug_pregrasp_alignment_pub.publish(text)

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

        self.logger.info('Moving to a coarse pregrasp pose so the D405 can see the object.')
        pregrasp_success, object_xyz_base_link_now = self.move_to_simple_pregrasp_pose(
            grasp_target, object_xyz_base_link)
        if not pregrasp_success:
            return Trigger.Response(
                success=False,
                message='Pregrasp move failed (see log).'
            )

        if not self.start_stage2_tracking(object_xyz_base_link_now, grasp_target['width_m']):
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
        self.move_base = nv.MoveBase(self)
        # Scoped to this node only -- grasp_object.py is unaffected.
        self.set_parameters([Parameter('base_translate_velocity', value=PREGRASP_DRIVE_VELOCITY_MPS)])
        self.logger = self.get_logger()

        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter('debug_directory', '')
        self.debug_directory = self.get_parameter('debug_directory').value or None
        self.logger.info('Using the following directory for debugging files: {0}'.format(self.debug_directory))

        self.declare_parameter('dryrun', False)
        self.dryrun = self.get_parameter('dryrun').value

        self.joint_states_subscriber = self.create_subscription(
            JointState, '/stretch/joint_states', callback=self.joint_states_callback,
            qos_profile=1, callback_group=self.callback_group)

        self.gripper_aruco_subscriber = self.create_subscription(
            MarkerArray, '/gripper_camera/aruco/marker_array', self.fingertip_marker_array_callback,
            qos_profile=1, callback_group=self.callback_group)

        self.gripper_point_cloud_subscriber = self.create_subscription(
            PointCloud2, '/gripper_camera/depth/color/points', self.gripper_point_cloud_callback,
            qos_profile=1, callback_group=self.callback_group)

        self.debug_marker_pub = self.create_publisher(Marker, '/grasp_object_visual_servo/debug/tracked_object', 1)
        self.debug_marker_raw_pub = self.create_publisher(Marker, '/grasp_object_visual_servo/debug/tracked_object_raw', 1)
        self.debug_points_pub = self.create_publisher(Marker, '/grasp_object_visual_servo/debug/surviving_points', 1)
        self.debug_points_pre_height_pub = self.create_publisher(
            Marker, '/grasp_object_visual_servo/debug/points_before_height_filter', 1)
        self.debug_pregrasp_alignment_pub = self.create_publisher(
            Marker, '/grasp_object_visual_servo/debug/pregrasp_alignment', 1)

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
