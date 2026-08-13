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
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import hello_helpers.hello_misc as hm
import hello_helpers.fit_plane as fp
import stretch_funmap.manipulation_planning as mp
import stretch_funmap.navigate as nv

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

# Stage 1 head-scan pose. Deliberately NOT using ManipulationView.move_head()
# (stretch_funmap/manipulation_planning.py), which hardcodes pan=-1.8/tilt=-0.8 with no
# override parameters and is shared by funmap.py and clean_surface.py too -- changing it
# in place would affect those. ManipulationView.update() has no dependency on the
# specific head pose used (confirmed via code read: it's purely reactive to whatever
# point cloud/TF is current at call time), so it's safe to do our own head move here
# instead of calling move_head() at all.
HEAD_SCAN_PAN_RAD = - np.pi / 2.0
HEAD_SCAN_TILT_RAD = -0.8  # unchanged from ManipulationView.move_head()'s own default
HEAD_SCAN_SETTLE_TIME_S = 0.5  # matches move_head()'s own settle time

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
STAGE2_CONNECTIVITY_RADIUS_M = 0.01  # see connected_cluster_near_point()
STAGE2_CLUSTER_EXTENT_MARGIN_M = 0.03  # added to object_width_m/2, see connected_cluster_near_point()'s max_extent_m
STAGE2_FINGERTIP_EXCLUSION_RADIUS_M = 0.03
STAGE2_MIN_POINTS = 15
STAGE2_SMOOTHING_ALPHA = 0.3  # see smooth_estimate() -- placeholder, not yet tuned
STAGE2_MAX_DRIFT_FROM_ANCHOR_M = 0.08  # see anchor/leash check in stage2_tracking_tick()
STAGE2_TICK_PERIOD_S = 1.0 / 15.0
STAGE2_TEST_DURATION_S = 30.0  # Increment 2 only: auto-stop the log-only test loop

# Pregrasp rotate-then-drive tuning -- placeholders, not yet tuned on hardware. Fixes a
# bug where get_pregrasp_planar_translation() (which only ever drives the base along its
# own fixed forward axis, never rotates) blew past the wrist's real ~0.5m max extension
# for objects offset mostly sideways (large base_link y) rather than in front of the
# robot -- see grasp-demo-status.md, Increment 3 step 1 postmortem.
PREGRASP_ROTATE_STOPPING_DISTANCE_M = 0.45  # planar distance left to the object after
                                             # the new drive step, before the existing
                                             # pregrasp reach math runs
PREGRASP_ROTATE_MAX_DRIVE_FORWARD_M = 1.0   # defensive clamp; Stage 1's own
                                             # max_object_planar_distance_m=1.0 already
                                             # bounds this in practice

# Simplified, non-rotating pregrasp move -- see move_to_simple_pregrasp_pose().
# Placeholders, not yet tuned on hardware. Known accepted limitation: this approach
# cannot reach an object offset sideways (base_link y) by more than the arm's real
# reach, since driving forward/backward alone cannot reduce a sideways offset -- see
# grasp-demo-status.md.
PREGRASP_LIFT_OFFSET_FROM_OBJECT_M = 0.20  # added to the object's height; negative = below
PREGRASP_ARM_EXTENSION_STANDOFF_M = 0.45    # stop the wrist extension this far short of the object
PREGRASP_FIXED_WRIST_YAW_RAD = 0.0          # single fixed pregrasp wrist yaw, not computed per-object


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


def compute_bearing_to_object(object_xyz_base_link):
    """Bearing (radians) from base_link's current +x axis to the object,
    using only the planar (x, y) components of object_xyz_base_link.

    Needed because get_pregrasp_planar_translation() (stretch_funmap's
    ManipulationView, reused unmodified elsewhere in this file) only ever
    drives the base along its own fixed +x (forward) axis and assumes the
    object is already roughly in front of it -- an object offset mostly
    along -y (the arm's fixed telescoping-extension direction) can't have
    that offset reduced by driving forward at all, so the whole gap lands
    on wrist extension and can exceed the arm's ~0.5m physical max. This
    bearing is used to rotate the base to face the object first (see
    rotate_to_face_object()), so the existing pregrasp math then runs
    within the range it was actually designed for.

    Positive is a counterclockwise (left) turn, matching base_link's
    standard convention and MoveBase.turn()'s relative-angle contract --
    no angle-wrapping needed since atan2's own (-pi, pi] range is already
    the shortest-path relative turn. Pure function.
    """
    x, y = object_xyz_base_link[0], object_xyz_base_link[1]
    return float(np.arctan2(y, x))


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


def connected_cluster_near_point(points_xyz, seed_point, connectivity_radius_m, max_extent_m=None):
    """Grow a connected cluster of points starting from whichever point is
    closest to seed_point, via simple nearest-neighbor connectivity
    (flood-fill/BFS): a point joins the cluster if it's within
    connectivity_radius_m of any point already in the cluster.

    Why this is needed: found empirically across three test objects on
    hardware that the search-radius + height-above-surface filters alone
    are repeatedly pulled toward a stereo depth artifact (spurious
    readings at occlusion boundaries) sitting a few cm away from the real
    object -- a separate, physically disjoint cluster that happens to
    also be elevated and within the search sphere. A pure radius/height
    filter has no way to tell "same object" apart from "different nearby
    thing" -- connectivity does: as long as there's a real physical gap
    (no points bridging the two clusters), growing outward from wherever
    we're already tracking naturally excludes anything not actually
    connected to it, without needing to know which cluster is "bigger" or
    otherwise classify them.

    max_extent_m: if given, caps growth to points within max_extent_m of
    seed_point itself (not just of the growing frontier). Needed in
    practice: on real D405 data, connectivity alone is not sufficient --
    unlike the cleanly-separated-blob case this function was originally
    unit-tested against, real point clouds often have a thin, locally-dense
    "bridge" of points (e.g. along a continuous surface or edge) connecting
    what should be two separate clusters, so pure single-linkage
    (flood-fill) clustering chains across it and merges them anyway. Since
    Stage 1 already gives us the object's approximate size (width_m), that
    bounds how big the *real* object's cluster should be, and rejects
    anything the flood-fill reaches only by chaining past that size.

    No new dependency: FUNMAP's own connected-component logic
    (segment_max_height_image.py) operates on a labeled 2D grid via
    skimage, which doesn't apply to an unstructured 3D point cloud; this
    is a from-scratch, numpy-only equivalent for that case. Pure function.

    Returns the Mx3 array of points in the connected cluster containing
    the point nearest to seed_point (empty if points_xyz is empty).
    """
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
    """Local point-cloud centroid tracker (Stage 2's core per-tick update).

    points_xyz: Nx3 numpy array, camera frame.
    previous_estimate_xyz: 3-vector, current tracked position, camera frame.
    min_height_above_surface_m: if > 0, apply
      filter_points_above_local_surface() after the spatial radius filter --
      needed in practice: a spatial-radius-only filter is dominated by
      whatever flat surface the object sits on (many more surface points
      than object points within any reasonably-sized search sphere), which
      pulls the centroid toward the surface rather than the object.
    connectivity_radius_m: if > 0, apply connected_cluster_near_point() after
      the height filter -- needed in practice: a nearby, spatially disjoint
      stereo depth artifact can also pass the radius+height filters and pull
      the centroid toward it. Keeps only the cluster physically connected to
      wherever we're already tracking.
    cluster_max_extent_m: forwarded to connected_cluster_near_point() as
      max_extent_m -- bounds connectivity growth to the object's known
      approximate size, so a thin bridge of points can't chain the cluster
      into an unrelated region (see that function's docstring).
    exclude_points: optional list of 3-vectors (e.g. fingertip contact
      points) to exclude nearby points from (avoids the gripper's own
      fingers contaminating the object estimate).

    Returns (new_estimate_xyz_or_None, num_points_used, surviving_points,
    points_after_radius_filter). surviving_points is the actual filtered
    Nx3 array that fed the centroid (or that failed the min_points check)
    -- exposed for debugging/visualization, not used in the centroid math
    itself beyond what's already returned. points_after_radius_filter is
    the intermediate Nx3 array right after only the spatial radius filter,
    before the height-above-surface / connectivity / fingertip-exclusion
    filters -- exposed so a debug diagnostic can show whether the object's
    real points are present at that stage and only get discarded by a
    later filter (vs. never being captured by the radius filter at all).
    Pure function -- no ROS/self state -- so it's unit-testable with
    synthetic arrays.
    """
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
            self.move_base.forward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)
        else:
            self.move_base.backward(forward_m, detect_obstacles=False, tolerance_distance_m=tolerance_distance_m)

    def rotate_to_face_object(self, object_xyz_base_link):
        """Rotate the base in place to face the object directly, before
        the existing pregrasp reach math runs (see move_to_pregrasp_pose()
        and compute_bearing_to_object() for why this is needed -- that
        math only ever drives the base along its own fixed forward axis,
        so a large sideways offset has to be removed by rotating, not
        translating). Uses MoveBase.turn() (stretch_funmap.navigate),
        which takes a relative angle in radians and blocks internally with
        its own retry/tolerance logic -- no additional setup needed.
        Skipped entirely under dryrun, matching move_to_pregrasp_pose()'s
        convention. A turn that doesn't fully converge is logged but
        treated as non-fatal: the existing wrist-extension correction in
        move_to_pregrasp_pose() absorbs whatever small residual offset is
        left over, the same mechanism this whole fix already relies on.
        Known accepted risk: MoveBase.turn() has no obstacle awareness,
        same as self.drive()'s existing detect_obstacles=False convention
        (matching grasp_object.py's own choice), now applied to a rotation
        too -- see grasp-demo-status.md.
        """
        if self.dryrun:
            self.logger.info('dryrun: skipping the pregrasp rotate-to-face-object step.')
            return True

        theta_rad = compute_bearing_to_object(object_xyz_base_link)
        self.logger.info(
            'Pregrasp: rotating {0:.3f} rad ({1:.1f} deg) to face the object.'.format(
                theta_rad, np.degrees(theta_rad)))
        at_goal = self.move_base.turn(theta_rad)
        if not at_goal:
            self.logger.warning(
                'Pregrasp: rotate-to-face-object turn did not fully converge to its target angle.')
        return at_goal

    def drive_to_pregrasp_range(self, grasp_target):
        """Second half of the new pregrasp step: after
        rotate_to_face_object() has (approximately) turned the base to
        face the object, re-derive the object's position in the
        now-current base_link frame -- reusing the exact same pixel-space
        data Stage 1 already computed in find_grasp_target(); the
        underlying max-height-image pixel location doesn't change, and the
        live TF lookup inside get_pix_in_frame() naturally reflects the
        robot's new, rotated pose. Then drive the base so only
        PREGRASP_ROTATE_STOPPING_DISTANCE_M of planar distance to the
        object remains, leaving a comfortable, un-clamped reach for the
        existing (unmodified) get_pregrasp_planar_translation() call right
        after this. Skipped entirely under dryrun.
        """
        if self.dryrun:
            self.logger.info('dryrun: skipping the pregrasp drive-to-range step.')
            return True

        h = self.manipulation_view.max_height_im
        xyz_pix = [grasp_target['location_xy_pix'][0],
                   grasp_target['location_xy_pix'][1],
                   grasp_target['location_z_pix']]
        object_xyz_base_link_now = h.get_pix_in_frame(xyz_pix, 'base_link', self.tf2_buffer)

        forward_m = object_xyz_base_link_now[0] - PREGRASP_ROTATE_STOPPING_DISTANCE_M
        forward_m = max(min(forward_m, PREGRASP_ROTATE_MAX_DRIVE_FORWARD_M), -PREGRASP_ROTATE_MAX_DRIVE_FORWARD_M)
        self.logger.info(
            'Pregrasp: after rotating, object now at x={0:.3f}, y={1:.3f} m (base_link); '
            'driving {2:.3f} m to leave a {3:.3f} m standoff.'.format(
                object_xyz_base_link_now[0], object_xyz_base_link_now[1],
                forward_m, PREGRASP_ROTATE_STOPPING_DISTANCE_M))
        self.drive(forward_m)
        return True

    def move_to_pregrasp_pose(self, grasp_target):
        """Coarse, position-mode move -- reusing grasp_object.py's own
        pregrasp logic (ManipulationView.get_pregrasp_lift/get_pregrasp_yaw/
        get_pregrasp_planar_translation, unmodified) -- to get the arm and
        gripper (and therefore the D405) out of its stowed position and
        roughly facing the object. Stage 2's own closed-loop tracking then
        takes over from there for the fine approach; this step only needs
        to get the object inside the D405's field of view, not precisely
        positioned. Skipped entirely under dryrun, same as grasp_object.py
        treats its own pregrasp/drive steps.
        """
        if self.dryrun:
            self.logger.info('dryrun: skipping the pregrasp move.')
            return True

        max_lift_m = 1.09
        min_extension_m = 0.01
        max_extension_m = 0.5

        if self.lift_position is None:
            self.logger.error('move_to_pregrasp_pose: lift position unavailable.')
            return False

        pregrasp_lift_m = self.manipulation_view.get_pregrasp_lift(grasp_target, self.tf2_buffer)
        if self.tool == "tool_stretch_dex_wrist":
            pregrasp_lift_m += 0.02
        self.logger.info('Raise tool to pregrasp height.')
        lift_to_pregrasp_m = max(self.lift_position + pregrasp_lift_m, 0.1)
        lift_to_pregrasp_m = min(lift_to_pregrasp_m, max_lift_m)
        self.move_to_pose({'joint_lift': lift_to_pregrasp_m})

        if self.tool == "tool_stretch_dex_wrist":
            self.logger.info('Rotate pitch/roll for grasping.')
            self.move_to_pose({'joint_wrist_pitch': -0.3, 'joint_wrist_roll': 0.0})

        pregrasp_yaw = self.manipulation_view.get_pregrasp_yaw(grasp_target, self.tf2_buffer)
        self.logger.info('Rotate the gripper for grasping: pregrasp_yaw = {0:.2f} rad'.format(pregrasp_yaw))
        self.move_to_pose({'joint_wrist_yaw': pregrasp_yaw})

        self.logger.info('Open the gripper.')
        self.move_to_pose({'gripper_aperture': 0.07})

        pregrasp_mobile_base_m, pregrasp_wrist_extension_m = self.manipulation_view.get_pregrasp_planar_translation(
            grasp_target, self.tf2_buffer)
        self.logger.info(
            'Drive to pregrasp location: base={0:.3f} m, wrist_extension_delta={1:.3f} m'.format(
                pregrasp_mobile_base_m, pregrasp_wrist_extension_m))
        self.drive(pregrasp_mobile_base_m)

        if pregrasp_wrist_extension_m > 0.0:
            extension_m = max(self.wrist_position + pregrasp_wrist_extension_m, min_extension_m)
            extension_m = min(extension_m, max_extension_m)
            self.logger.info('Extend tool toward the object.')
            self.move_to_pose({'wrist_extension': extension_m})
        else:
            self.logger.info('Negative wrist extension for pregrasp, so not extending or retracting.')

        return True

    def move_to_simple_pregrasp_pose(self, grasp_target, object_xyz_base_link):
        """Simplified, non-rotating pregrasp move -- replaces
        move_to_pregrasp_pose() (kept above, unused, for comparison) with
        four straightforward position-mode steps, each using Stage 1's
        object_xyz_base_link, re-derived after the base moves (the
        object's position relative to the moving base_link frame changes
        as the robot drives, so re-look it up via the same
        get_pix_in_frame() call Stage 1 already used, rather than trusting
        the pre-drive value):
          1. Drive forward/backward until the object's base_link x is
             close to zero -- the robot ends up roughly alongside the
             object.
          2. Move the lift to the object's height plus
             PREGRASP_LIFT_OFFSET_FROM_OBJECT_M (currently negative, i.e.
             below the object's height).
          3. Extend the wrist toward the object using its (re-derived)
             base_link y offset, stopping PREGRASP_ARM_EXTENSION_STANDOFF_M
             short.
          4. Rotate the wrist to a single fixed, always-the-same pregrasp
             yaw (no per-object yaw calculation).

        Does not rotate the base at all, by design (for now). Known
        accepted limitation: an object offset sideways (base_link y) by
        more than the arm's real reach cannot be reached this way, since
        driving forward/backward alone cannot reduce a sideways offset --
        see grasp-demo-status.md for the full reasoning. Skipped entirely
        under dryrun.
        """
        if self.dryrun:
            self.logger.info('dryrun: skipping the simple pregrasp move.')
            return True

        max_lift_m = 1.09
        min_extension_m = 0.01
        max_extension_m = 0.5

        if self.lift_position is None or self.wrist_position is None:
            self.logger.error('move_to_simple_pregrasp_pose: lift/wrist position unavailable.')
            return False

        # Step 1: drive so the object's base_link x is close to zero.
        forward_m = object_xyz_base_link[0]
        forward_m = max(min(forward_m, PREGRASP_ROTATE_MAX_DRIVE_FORWARD_M), -PREGRASP_ROTATE_MAX_DRIVE_FORWARD_M)
        self.logger.info('Pregrasp (simple): driving {0:.3f} m to align x.'.format(forward_m))
        self.drive(forward_m)

        # Re-derive the object's position after the drive -- the underlying
        # pixel-space data doesn't change, but the live TF lookup inside
        # get_pix_in_frame() reflects wherever the robot actually ended up.
        h = self.manipulation_view.max_height_im
        xyz_pix = [grasp_target['location_xy_pix'][0],
                   grasp_target['location_xy_pix'][1],
                   grasp_target['location_z_pix']]
        object_xyz_base_link_now = h.get_pix_in_frame(xyz_pix, 'base_link', self.tf2_buffer)
        self.logger.info(
            'Pregrasp (simple): after driving, object now at x={0:.3f}, y={1:.3f}, z={2:.3f} m (base_link).'.format(
                object_xyz_base_link_now[0], object_xyz_base_link_now[1], object_xyz_base_link_now[2]))

        # Step 2: move the lift to the object's height, offset by
        # PREGRASP_LIFT_OFFSET_FROM_OBJECT_M.
        target_lift_m = object_xyz_base_link_now[2] + PREGRASP_LIFT_OFFSET_FROM_OBJECT_M
        target_lift_m = max(min(target_lift_m, max_lift_m), 0.1)
        self.logger.info('Pregrasp (simple): moving lift to {0:.3f} m.'.format(target_lift_m))
        self.move_to_pose({'joint_lift': target_lift_m})

        # Step 3: extend the wrist toward the object using its y offset.
        extension_m = abs(object_xyz_base_link_now[1]) - PREGRASP_ARM_EXTENSION_STANDOFF_M
        extension_m = max(min(extension_m, max_extension_m), min_extension_m)
        self.logger.info('Pregrasp (simple): extending wrist to {0:.3f} m.'.format(extension_m))
        self.move_to_pose({'wrist_extension': extension_m})

        # Step 4: fixed pregrasp wrist yaw, same every time.
        self.logger.info('Pregrasp (simple): rotating wrist yaw to fixed {0:.3f} rad.'.format(
            PREGRASP_FIXED_WRIST_YAW_RAD))
        self.move_to_pose({'joint_wrist_yaw': PREGRASP_FIXED_WRIST_YAW_RAD})

        self.logger.info('Open the gripper.')
        self.move_to_pose({'gripper_aperture': 0.07})

        return True

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
        # Kept in base_link (a stable frame, unlike the D405 camera frame,
        # which moves) so it can be re-projected into wherever the camera
        # currently is, every tick -- the anchor/leash check in
        # stage2_tracking_tick() uses this to prevent the tracker from
        # drifting away from the object with nothing pulling it back.
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

        # The search is centered on the Stage 1 anchor -- re-projected into
        # the camera's *current* frame every tick via a fresh TF lookup --
        # rather than on wherever the tracker's own previous output landed.
        # Found empirically: a purely self-referential search (centering on
        # its own last result) can lock onto a nearby-but-wrong cluster on
        # the very first successful tick (e.g. a partial/edge view caught
        # mid-sweep while manually moving the camera into place) and then
        # never escape it -- the connected-component + extent-cap filter
        # (by design) won't jump to a different, disconnected cluster even
        # once the camera settles onto a clear view of the real object, and
        # if the bad lock happens to sit within the anchor leash's
        # tolerance, the leash never trips to correct it either. Anchoring
        # the search itself in the known, kinematically-grounded Stage 1
        # location removes that failure mode: every tick gets an
        # independent chance to (re)find the real object, rather than
        # trusting its own potentially-biased prior result.
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

        # Diagnostic: visualize exactly which points survived filtering
        # and fed the centroid, regardless of whether tracking succeeded
        # this tick -- shows directly whether the tracker is looking at
        # the real object or some other nearby cluster.
        self.publish_debug_points(surviving_points)

        # Diagnostic: visualize the intermediate point set right after
        # only the spatial radius filter -- before height-above-surface /
        # connectivity / fingertip exclusion run. Lets us tell whether the
        # object's real points are present here and get discarded by a
        # later filter stage, vs. never being captured by the radius
        # filter at all.
        self.publish_debug_points(points_after_radius_filter, self.debug_points_pre_height_pub, rgb=(0.0, 1.0, 0.0))

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

        # Safety net, kept even though the search is now anchor-centered:
        # cap how far the *smoothed* estimate is allowed to drift from the
        # anchor before being reported, in case the smoothing history
        # itself has lagged (e.g. after several ticks of a genuinely
        # moving target). Should rarely trigger now, since new_estimate
        # itself can't come from further than the search radius away from
        # the anchor.
        drift_from_anchor_m = float(np.linalg.norm(smoothed_estimate - anchor_xyz_camera_now))
        if drift_from_anchor_m > STAGE2_MAX_DRIFT_FROM_ANCHOR_M:
            self.logger.warning(
                'Stage 2 tick: smoothed estimate drifted {0:.3f} m from the Stage 1 anchor '
                '(max {1:.3f} m) -- resetting to the anchor.'.format(
                    drift_from_anchor_m, STAGE2_MAX_DRIFT_FROM_ANCHOR_M))
            smoothed_estimate = anchor_xyz_camera_now

        # Internal tracking state (used only to damp jitter via
        # smooth_estimate() next tick, no longer used to center the
        # search -- that's always the freshly re-projected anchor above).
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

        # Diagnostic: publish both the raw (smoothed, pre-push) centroid
        # and the reported (post-push) position as separate markers, so
        # RViz can show directly whether a lateral offset from the object
        # is already present before push_along_viewing_ray() runs (a
        # point-filtering/centroid issue) or only appears after it (a
        # push-math issue).
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
        if not self.move_to_simple_pregrasp_pose(grasp_target, object_xyz_base_link):
            return Trigger.Response(
                success=False,
                message='Pregrasp move failed (see log).'
            )

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
        self.move_base = nv.MoveBase(self)
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