#! /usr/bin/env python3

import array
import copy
from functools import cache
import cv2
import numpy as np
import threading

from sensor_msgs.msg._compressed_image import CompressedImage
from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.enums.actuators import Actuators
from stretch_mujoco.enums.stretch_sensors import StretchSensors
from stretch_mujoco.enums.stretch_cameras import CameraSettings, StretchCameras
from stretch_mujoco.robocasa_gen import (
    layout_from_str,
    style_from_str,
    model_generation_wizard,
    get_styles,
    layouts,
)

from rclpy.qos import QoSProfile, ReliabilityPolicy
from stretch_core.rwlock import RWLock
from stretch_mujoco_driver.joint_trajectory_server import JointTrajectoryAction
import tf2_ros
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.duration import Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from rosgraph_msgs.msg import Clock

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge


from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField, Joy
from std_msgs.msg import Bool, String, Float64MultiArray

from hello_helpers.joint_qpos_conversion import SE3_dw3_sg3_Idx
from hello_helpers.joint_qpos_conversion import get_Idx
from hello_helpers.gripper_conversion import GripperConversion
from hello_helpers.gamepad_conversion import (
    unpack_joy_to_gamepad_state,
    unpack_gamepad_state_to_joy,
    get_default_joy_msg,
)

# from .joint_trajectory_server import JointTrajectoryAction
from builtin_interfaces.msg import Time as TimeMsg
from ament_index_python.packages import get_package_share_path


from rclpy import time as rclpyTime


DEFAULT_TIMEOUT = 0.5
DEFAULT_GOAL_TIMEOUT = 10.0
DEFAULT_ROBOCASA_TASK = "PnPCounterToCab"
DEFAULT_ACTION_SERVER_HZ = 30.0
DEFAULT_JOINT_STATE_HZ = 30.0
DEFAULT_SIM_TOOL = "eoa_wrist_dw3_tool_sg3"


class StretchMujocoDriver(Node):

    def __init__(self):

        super().__init__("stretch_mujoco_driver")
        self.declare_parameter("use_cameras", False)
        self.declare_parameter("use_mujoco_viewer", True)
        self.declare_parameter("use_robocasa", True)
        self.declare_parameter("robocasa_task", DEFAULT_ROBOCASA_TASK)
        self.declare_parameter("robocasa_layout", None)
        self.declare_parameter("robocasa_style", None)

        use_cameras = self.get_parameter("use_cameras").value
        use_mujoco_viewer = self.get_parameter("use_mujoco_viewer").value

        model = None

        use_robocasa = self.get_parameter("use_robocasa").value
        if use_robocasa:
            robocasa_task: str | None = self.get_parameter("robocasa_task").value
            robocasa_layout = self.get_parameter("robocasa_layout").value
            robocasa_style = self.get_parameter("robocasa_style").value

            if isinstance(robocasa_layout, str):
                # Convert robocasa_layout to int
                if robocasa_layout.isnumeric():
                    robocasa_layout = int(robocasa_layout)
                elif robocasa_layout == "Random":
                    robocasa_layout = np.random.choice(range(len(layouts)))
                else:
                    robocasa_layout = layout_from_str(robocasa_layout)
            elif robocasa_layout is None:
                robocasa_layout = -1

            if isinstance(robocasa_style, str):
                # Convert robocasa_style to int
                if robocasa_style.isnumeric():
                    robocasa_style = int(robocasa_style)
                elif robocasa_style == "Random":
                    robocasa_style = np.random.choice(range(len(get_styles())))
                else:
                    robocasa_style = style_from_str(robocasa_style)
            elif robocasa_style is None:
                robocasa_style = -1

            model, xml, objects_info = model_generation_wizard(
                task=robocasa_task or DEFAULT_ROBOCASA_TASK,
                layout=robocasa_layout,
                style=robocasa_style,
            )

        sim = StretchMujocoSimulator(
            model=model,
            camera_hz=10,
            cameras_to_use=(
                StretchCameras.all() if use_cameras else StretchCameras.none()
            ),
        )

        sim.start(headless=not use_mujoco_viewer)

        self.sim = sim

        # Initialize calibration offsets
        self.head_tilt_calibrated_offset_rad = 0.0
        self.head_pan_calibrated_offset_rad = 0.0

        # Initialize backlash state
        self.backlash_state = {
            "head_tilt_looking_up": False,
            "head_pan_looked_left": False,
            "wrist_extension_retracted": False,
        }

        # Initialize backlash offsets
        self.head_pan_calibrated_looked_left_offset_rad = 0.0
        self.head_tilt_calibrated_looking_up_offset_rad = 0.0
        self.wrist_extension_calibrated_retracted_offset_m = 0.0
        self.head_tilt_backlash_transition_angle_rad = -0.4

        self.gripper_conversion = GripperConversion()

        self.robot_stop_lock = threading.Lock()

        self.robot_mode_rwlock = RWLock()
        self.robot_mode = None
        self.control_modes = ["position", "navigation", "trajectory", "gamepad"]
        self.prev_runstop_state = None  # helps track if runstop state has changed

        self.voltage_history = []
        self.charging_state_history = [BatteryState.POWER_SUPPLY_STATUS_UNKNOWN] * 10
        self.charging_state = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        self.received_gamepad_joy_msg = get_default_joy_msg()

        self.streaming_position_activated = False

        self.bridge = CvBridge()

        self.ros_setup()

    def set_gamepad_motion_callback(self, joy):
        self.robot_mode_rwlock.acquire_read()
        if self.robot_mode != "gamepad":
            self.get_logger().error(
                "{0} Stretch Driver must be in gamepad mode to "
                "receive a Joy msg on gamepad_joy topic. "
                "Current mode = {1}.".format(self.node_name, self.robot_mode)
            )
            self.robot_mode_rwlock.release_read()
            return
        self.received_gamepad_joy_msg = joy
        self.last_gamepad_joy_time = self.get_clock().now()
        self.robot_mode_rwlock.release_read()

    # MOBILE BASE VELOCITY METHODS ############

    def set_mobile_base_velocity_callback(self, twist):
        self.robot_mode_rwlock.acquire_read()
        if self.robot_mode != "navigation":
            self.get_logger().error(
                "{0} action server must be in navigation mode to "
                "receive a twist on cmd_vel. "
                "Current mode = {1}.".format(self.node_name, self.robot_mode)
            )
            self.robot_mode_rwlock.release_read()
            return
        self.linear_velocity_mps = twist.linear.x
        self.angular_velocity_radps = twist.angular.z
        self.last_twist_time = self.get_clock().now()
        self.robot_mode_rwlock.release_read()

    def set_robot_streaming_position_callback(self, msg):
        self.robot_mode_rwlock.acquire_read()
        if not self.streaming_position_activated:
            self.get_logger().error(
                "Streaming position is not activated."
                " Please activate streaming position to receive command to joint_position_cmd."
            )
            self.robot_mode_rwlock.release_read()
            return

        if not self.robot_mode in ["position", "navigation"]:
            self.get_logger().error(
                "{0} must be in position or navigation mode with streaming_position activated "
                "enabled to receive command to joint_position_cmd. "
                "Current mode = {1}.".format(self.node_name, self.robot_mode)
            )
            self.robot_mode_rwlock.release_read()
            return

        qpos = msg.data
        self.move_to_position(qpos)
        self.robot_mode_rwlock.release_read()

    def move_to_position(self, qpos):
        try:
            Idx: SE3_dw3_sg3_Idx = get_Idx(DEFAULT_SIM_TOOL)  # type: ignore

            if len(qpos) != Idx.num_joints:
                self.get_logger().error(
                    "Received qpos does not match the number of joints in the robot"
                )
                return
            self.sim.move_to(Actuators.arm, qpos[Idx.ARM])
            self.sim.move_to(Actuators.lift, qpos[Idx.LIFT])
            self.sim.move_to(Actuators.wrist_yaw, qpos[Idx.WRIST_YAW])
            self.sim.move_to(Actuators.wrist_pitch, qpos[Idx.WRIST_PITCH])
            self.sim.move_to(Actuators.wrist_roll, qpos[Idx.WRIST_ROLL])
            self.sim.move_to(Actuators.head_pan, qpos[Idx.HEAD_PAN])
            self.sim.move_to(Actuators.head_tilt, qpos[Idx.HEAD_TILT])

            is_base_translate_command =  abs(qpos[Idx.BASE_TRANSLATE]) > 0.0
            is_base_rotate_command =  abs(qpos[Idx.BASE_ROTATE]) > 0.0
            if self.robot_mode != "position" and ( is_base_translate_command or is_base_rotate_command):
                self.get_logger().error(
                    "Cannot set base position when not in position mode."
                )
                
            elif abs(qpos[Idx.BASE_TRANSLATE]) > 0.0 and self.robot_mode == "position":
                self.sim.move_by(Actuators.base_translate, qpos[Idx.BASE_TRANSLATE])
            elif abs(qpos[Idx.BASE_ROTATE]) > 0.0 and self.robot_mode == "position":
                self.sim.move_by(Actuators.base_rotate, qpos[Idx.BASE_ROTATE])

            pos = self.gripper_conversion.finger_to_robotis(qpos[Idx.GRIPPER])
            self.sim.move_to(Actuators.gripper, pos)

            for actuator in [
                Actuators.arm,
                Actuators.lift,
                Actuators.wrist_pitch,
                Actuators.wrist_roll,
                Actuators.wrist_yaw,
                Actuators.head_pan,
                Actuators.head_tilt,
                Actuators.gripper,
            ]:
                succeeded = self.sim.wait_until_at_setpoint(actuator)
                if not succeeded:
                    raise Exception(
                        f"{actuator} failed to move to {self.sim.data_proxies.get_command().move_to[actuator.name]}"
                    )
                
            if self.robot_mode == "position":
                for actuator in [
                    Actuators.base_translate,
                    Actuators.base_rotate,
                ]:
                    succeeded = self.sim.wait_while_is_moving(actuator)
                    if not succeeded:
                        raise Exception(
                            f"{actuator} failed to move to {self.sim.data_proxies.get_command().move_to[actuator.name]}"
                        )

            self.get_logger().info(f"Moved to position qpos: {qpos}")
        except Exception as e:
            self.get_logger().error("Failed to move to position: {0}".format(e))

    def command_mobile_base_velocity_and_publish_state(self):

        self.robot_mode_rwlock.acquire_read()

        # During gamepad mode, the robot can be controlled with provided gamepad dongle plugged into the robot
        # Or a Joy message type could also be published which can be used for controlling robot with an remote gamepad.
        # The Joy message should follow the format described in gamepad_conversion.py
        # if self.robot_mode == 'gamepad':
        #     time_since_last_joy = self.get_clock().now() - self.last_gamepad_joy_time
        #     if time_since_last_joy < self.timeout:
        #         self.gamepad_teleop.do_motion(unpack_joy_to_gamepad_state(self.received_gamepad_joy_msg),robot=self.robot)
        #     else:
        #         self.gamepad_teleop.do_motion(robot=self.robot)
        # else:
        #     self.gamepad_teleop.update_gamepad_state(self.robot) # Update gamepad input readings within gamepad_teleop instance

        # Set new mobile base velocities
        if self.robot_mode == "navigation":
            time_since_last_twist = self.get_clock().now() - self.last_twist_time
            if time_since_last_twist < self.timeout:
                self.sim.set_base_velocity(
                    self.linear_velocity_mps, self.angular_velocity_radps
                )
            elif time_since_last_twist < Duration(seconds=self.timeout_s + 1.0):  # type: ignore
                # self.sim.set_base_velocity(0.0, 0.0)
                self.sim.move_by(Actuators.base_translate, 0.0)
            else:
                self.sim.set_base_velocity(0.0, 0.0)

        # get copy of the current robot status
        robot_status = self.sim.pull_status()

        self.get_logger().debug(robot_status.sim_to_real_time_ratio_msg)

        # Publish /clock for ROS to use sim time:
        seconds = int(robot_status.time)
        nanoseconds = int((robot_status.time - seconds) * 1e9)
        sim_time = rclpyTime.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
        self.clock_pub.publish(Clock(clock=sim_time))

        # Use node time for other topics, using sim time makes bad things happen.
        current_time = self.get_clock().now().to_msg()

        # obtain odometry
        # assign relevant base status to variables
        base_status = robot_status.base
        x = base_status.x
        y = base_status.y
        theta = base_status.theta
        x_vel = base_status.x_vel
        # y_vel = base_status.y_vel #TODO: implement y_vel in base_status
        y_vel = base_status.x_vel

        theta_vel = base_status.theta_vel

        q = quaternion_from_euler(0.0, 0.0, theta)

        if self.broadcast_odom_tf:
            # publish odometry via TF
            t = TransformStamped()
            t.header.stamp = current_time
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

            # This is important, otherwise all the joints are not transformed correctly. The alternative is to broadcast a static_transform, but that doesn't help if another node is trying to lookup transforms.
            self.tf_buffer.wait_for_transform_async(
                "base_link", "link_lift", rclpyTime.Time(seconds=0)
            )

            b = TransformStamped()
            b.header.stamp = current_time
            b.header.frame_id = self.base_frame_id
            b.child_frame_id = "base_footprint"
            self.tf_static_broadcaster.sendTransform(b)
            b.header.frame_id = "map"
            b.child_frame_id = self.odom_frame_id
            self.tf_static_broadcaster.sendTransform(b)

        # assign relevant arm status to variables
        arm_status = robot_status.arm
        if self.backlash_state["wrist_extension_retracted"]:
            arm_backlash_correction = self.wrist_extension_calibrated_retracted_offset_m
        else:
            arm_backlash_correction = 0.0

        pos_out = arm_status.pos + arm_backlash_correction
        vel_out = arm_status.vel
        # eff_out = arm_status.motor.effort_pct
        eff_out = 0.0

        lift_status = robot_status.lift
        pos_up = lift_status.pos
        vel_up = lift_status.vel
        # eff_up = lift_status.motor.effort_pct
        eff_up = 0.0

        # assign relevant wrist status to variables
        wrist_yaw_status = robot_status.wrist_yaw
        wrist_yaw_rad = wrist_yaw_status.pos
        wrist_yaw_vel = wrist_yaw_status.vel
        # wrist_yaw_effort = wrist_yaw_status.effort
        wrist_yaw_effort = 0.0

        wrist_pitch_status = robot_status.wrist_pitch
        wrist_pitch_rad = wrist_pitch_status.pos
        wrist_pitch_vel = wrist_pitch_status.vel
        # wrist_pitch_effort = wrist_pitch_status.effort
        wrist_pitch_effort = 0.0

        wrist_roll_status = robot_status.wrist_roll
        wrist_roll_rad = wrist_roll_status.pos
        wrist_roll_vel = wrist_roll_status.vel
        # wrist_roll_effort = wrist_roll_status.effort
        wrist_roll_effort = 0.0

        # assign relevant gripper status to variables
        # if 'stretch_gripper' in self.sim.end_of_arm.joints:
        #     gripper_status = robot_status['end_of_arm']['stretch_gripper']
        #     if GRIPPER_DEBUG:
        #         print('-----------------------')
        #         print('gripper_status[\'pos\'] =', gripper_status['pos'])
        #         print('gripper_status[\'pos_pct\'] =', gripper_status['pos_pct'])
        #     gripper_aperture_m, gripper_finger_rad, gripper_finger_effort, gripper_finger_vel = \
        #         self.gripper_conversion.status_to_all(gripper_status)
        #     if GRIPPER_DEBUG:
        #         print('gripper_aperture_m =', gripper_aperture_m)
        #         print('gripper_finger_rad =', gripper_finger_rad)
        #         print('-----------------------')

        # assign relevant head pan status to variables
        head_pan_status = robot_status.head_pan
        if self.backlash_state["head_pan_looked_left"]:
            pan_backlash_correction = self.head_pan_calibrated_looked_left_offset_rad
        else:
            pan_backlash_correction = 0.0

        head_pan_rad = (
            head_pan_status.pos
            + self.head_pan_calibrated_offset_rad
            + pan_backlash_correction
        )
        head_pan_vel = head_pan_status.vel
        # head_pan_effort = head_pan_status.effort
        head_pan_effort = 0.0

        # assign relevant head tilt status to variables
        head_tilt_status = robot_status.head_tilt
        if self.backlash_state["head_tilt_looking_up"]:
            tilt_backlash_correction = self.head_tilt_calibrated_looking_up_offset_rad
        else:
            tilt_backlash_correction = 0.0

        head_tilt_rad = (
            head_tilt_status.pos
            + self.head_tilt_calibrated_offset_rad
            + tilt_backlash_correction
        )
        head_tilt_vel = head_tilt_status.vel
        # head_tilt_effort = head_tilt_status.effort
        head_tilt_effort = 0.0

        ##################################################
        # publish homed status
        homed_status = Bool()
        homed_status.data = True
        self.homed_pub.publish(homed_status)

        # publish runstop event
        runstop_event = Bool()
        runstop_event.data = self.is_runstopped()
        self.runstop_event_pub.publish(runstop_event)

        # publish stretch_driver operation mode
        mode_msg = String()
        mode_msg.data = self.robot_mode
        self.mode_pub.publish(mode_msg)

        # publish end of arm tool
        tool_msg = String()
        tool_msg.data = "eoa_wrist_dw3_tool_sg3"
        self.tool_pub.publish(tool_msg)

        # publish streaming position status
        streaming_position_status = Bool()
        streaming_position_status.data = self.streaming_position_activated
        self.streaming_position_mode_pub.publish(streaming_position_status)

        # publish joint state for the arm
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = current_time
        # joint_arm_l3 is the most proximal and joint_arm_l0 is the
        # most distal joint of the telescoping arm model. The joints
        # are connected in series such that moving the most proximal
        # joint moves all the other joints in the global frame.
        joint_state.name = [
            "wrist_extension",
            "joint_lift",
            "joint_arm_l3",
            "joint_arm_l2",
            "joint_arm_l1",
            "joint_arm_l0",
        ]

        # set positions of the telescoping joints
        positions = [pos_out / 4.0 for i in range(4)]
        # set lift position
        positions.insert(0, pos_up)
        # set wrist_extension position
        positions.insert(0, pos_out)

        # set velocities of the telescoping joints
        velocities = [vel_out / 4.0 for i in range(4)]
        # set lift velocity
        velocities.insert(0, vel_up)
        # set wrist_extension velocity
        velocities.insert(0, vel_out)

        # set efforts of the telescoping joints
        efforts = [eff_out for i in range(4)]
        # set lift effort
        efforts.insert(0, eff_up)
        # set wrist_extension effort
        efforts.insert(0, eff_out)

        head_joint_names = ["joint_head_pan", "joint_head_tilt"]
        joint_state.name.extend(head_joint_names)

        positions.append(head_pan_rad)
        velocities.append(head_pan_vel)
        efforts.append(head_pan_effort)

        positions.append(head_tilt_rad)
        velocities.append(head_tilt_vel)
        efforts.append(head_tilt_effort)

        dex_wrist_attached = True

        if dex_wrist_attached:
            end_of_arm_joint_names = [
                "joint_wrist_yaw",
                "joint_wrist_pitch",
                "joint_wrist_roll",
            ]
            # if 'stretch_gripper' in self.sim.end_of_arm.joints:
            end_of_arm_joint_names = end_of_arm_joint_names + [
                "joint_gripper_finger_left",
                "joint_gripper_finger_right",
            ]
        else:
            # if 'stretch_gripper' in self.sim.end_of_arm.joints:
            end_of_arm_joint_names = [
                "joint_wrist_yaw",
                "joint_gripper_finger_left",
                "joint_gripper_finger_right",
            ]

        joint_state.name.extend(end_of_arm_joint_names)

        positions.append(wrist_yaw_rad)
        velocities.append(wrist_yaw_vel)
        efforts.append(wrist_yaw_effort)

        if dex_wrist_attached:
            positions.append(wrist_pitch_rad)
            velocities.append(wrist_pitch_vel)
            efforts.append(wrist_pitch_effort)

            positions.append(wrist_roll_rad)
            velocities.append(wrist_roll_vel)
            efforts.append(wrist_roll_effort)

            # Left Finger
            positions.append(robot_status.gripper.pos)
            velocities.append(robot_status.gripper.vel)
            efforts.append(0.0)
            # Right Finger
            positions.append(robot_status.gripper.pos)
            velocities.append(robot_status.gripper.vel)
            efforts.append(0.0)

        # set joint_state
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self.joint_state_pub.publish(joint_state)

        ##################################################
        # publish IMU sensor data
        sensor_status = self.sim.pull_sensor_data()

        accel_status = sensor_status.get_data(StretchSensors.base_accel)
        gyro_status = sensor_status.get_data(StretchSensors.base_gyro)
        ax = accel_status[0]
        ay = accel_status[1]
        az = accel_status[2]
        gx = gyro_status[0]
        gy = gyro_status[1]
        # gz = gyro_status[2]
        # qw = gyro_status[3]
        # qx =  gyro_status[4]
        # qy = gyro_status[5]
        # qz = gyro_status[6]
        gz = qw = qx = qy = qz = 0.0  # TODO

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = "imu_mobile_base"
        i.angular_velocity.x = gx
        i.angular_velocity.y = gy
        i.angular_velocity.z = gz

        i.orientation.w = qw
        i.orientation.x = qx
        i.orientation.y = qy
        i.orientation.z = qz

        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_mobile_base_pub.publish(i)

        m = MagneticField()
        m.header.stamp = current_time
        m.header.frame_id = "imu_mobile_base"
        self.magnetometer_mobile_base_pub.publish(m)

        # accel_status = robot_status.wacc
        # ax = accel_status.ax
        # ay = accel_status.ay
        # az = accel_status.az
        ax = ay = az = 0.0

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = "accel_wrist"
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_wrist_pub.publish(i)

        # publish odometry via the odom topic
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = x_vel
        odom.twist.twist.linear.y = y_vel
        odom.twist.twist.angular.z = theta_vel
        self.odom_pub.publish(odom)

        ##################################################
        # Publish Stretch Gamepad status
        # b = Bool()
        # b.data = True if self.gamepad_teleop.is_gamepad_dongle else False
        # self.is_gamepad_dongle_pub.publish(b)
        # j = unpack_gamepad_state_to_joy(self.gamepad_teleop.controller_state)
        # j.header.stamp = current_time
        # self.gamepad_state_pub.publish(j)

        self.robot_mode_rwlock.release_read()
        # must happen after the read release, otherwise the write lock in change_mode() will cause a deadlock
        if (self.prev_runstop_state == None and runstop_event.data) or (
            self.prev_runstop_state != None
            and runstop_event.data != self.prev_runstop_state
        ):
            self.runstop_the_robot(runstop_event.data, just_change_mode=True)

        self.prev_runstop_state = runstop_event.data

        self.publish_camera_and_lidar(current_time=current_time)

    def publish_camera_and_lidar(self, current_time: TimeMsg | None = None):

        current_time = current_time or self.get_clock().now().to_msg()

        sensor_status = self.sim.pull_sensor_data()

        try:
            lidar_data = sensor_status.get_data(StretchSensors.base_lidar)

            self.laser_scan_pub.publish(
                create_laser_scan_msg(
                    lidar_data, timestamp=current_time, frame_id="laser"
                )
            )
        except ValueError:
            ...  # Lidar is disabled, get_data() throws a ValueError

        camera_data = self.sim.pull_camera_data()
        for camera, frame in camera_data.get_all(
            auto_rotate=False, auto_correct_rgb=True
        ).items():
            header = Header()
            header.frame_id = get_camera_frame(camera)
            header.stamp = current_time

            ros_image = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8" if not camera.is_depth else "32FC1",
                header=header,
            )
            self.camera_publishers[camera.name].publish(ros_image)

            settings: CameraSettings = camera.initial_camera_settings
            camera_info = create_camera_info(
                camera_settings=settings,
                frame_id=header.frame_id,
                timestamp=current_time,
            )
            self.camera_info_publishers[camera.name].publish(camera_info)

            if camera.is_depth:
                ros_image_compressed = compress_depth_image(frame)
            else:
                ros_image_compressed: CompressedImage = (
                    self.bridge.cv2_to_compressed_imgmsg(frame, "png")
                )
            ros_image_compressed.header.frame_id = get_camera_frame(camera)
            self.camera_compressed_publishers[camera.name].publish(ros_image_compressed)

            if camera.is_depth:
                if camera == StretchCameras.cam_d405_depth:
                    pointcloud_msg = create_pointcloud_rgb_msg(
                        camera_info_msg=camera_info,
                        rgb_image=camera_data.get_camera_data(
                            StretchCameras.cam_d405_rgb
                        ),
                        depth_image=frame,
                    )
                elif camera == StretchCameras.cam_d435i_depth:
                    pointcloud_msg = create_pointcloud_rgb_msg(
                        camera_info_msg=camera_info,
                        rgb_image=camera_data.get_camera_data(
                            StretchCameras.cam_d435i_rgb, auto_rotate=False
                        ),
                        depth_image=camera_data.get_camera_data(
                            StretchCameras.cam_d435i_depth, auto_rotate=False
                        ),
                    )
                else:
                    pointcloud_msg = create_pointcloud_msg(camera_info, frame)
                self.pointcloud_publishers[camera.name].publish(pointcloud_msg)

    # CHANGE MODES ################
    def change_mode(self, new_mode, code_to_run=None):
        self.robot_mode_rwlock.acquire_write()

        self.robot_mode = new_mode

        if code_to_run:
            code_to_run()

        self.get_logger().info(f"Changed to mode = {self.robot_mode}")
        self.robot_mode_rwlock.release_write()

    def turn_on_navigation_mode(self):
        # Navigation mode enables mobile base velocity control via
        # cmd_vel, and disables position-based control of the mobile
        # base.
        def code_to_run():
            self.linear_velocity_mps = 0.0
            self.angular_velocity_radps = 0.0

        self.change_mode("navigation", code_to_run)
        return True, "Now in navigation mode."

    def turn_on_position_mode(self):
        # Position mode enables mobile base translation and rotation
        # using position control with sequential incremental rotations
        # and translations. It also disables velocity control of the
        # mobile base. It does not update the virtual prismatic
        # joint. The frames associated with 'floor_link' and
        # 'base_link' become identical in this mode.
        def code_to_run():
            # self.sim.base.enable_pos_incr_mode()
            ...

        self.change_mode("position", code_to_run)
        return True, "Now in position mode."

    def turn_on_trajectory_mode(self):
        # Trajectory mode is able to execute plans from
        # high level planners like MoveIt2. These planners
        # send whole robot waypoint trajectories to the
        # joint trajectory action server, and the underlying
        # Python interface to the robot (Stretch Body) executes
        # the trajectory, respecting each waypoints' time_from_start
        # attribute of the trajectory_msgs/JointTrajectoryPoint
        # message. This allows coordinated motion of the base + arm.
        raise NotImplementedError(
            "Trajectory Mode is not yet supported in StretchMujocoDriver."
        )

        def code_to_run():
            try:
                self.sim.stop_trajectory()
            except NotImplementedError as e:
                return False, str(e)
            self.sim.base.first_step = True
            self.sim.base.pull_status()

        self.change_mode("trajectory", code_to_run)
        return True, "Now in trajectory mode."

    def turn_on_gamepad_mode(self):
        # Gamepad mode enables the provided gamepad with stretch
        # to control the robot motions. If the gamepad USB dongle is plugged out
        # the robot would stop making any motions in this mode and could plugged in back in reltime.
        # Alternatively in this mode, stretch driver also listens to `gamepad_joy` topic
        # for valid Joy type message from a remote gamepad to control stretch.
        # The Joy message format is described in the gamepad_conversion.py
        raise NotImplementedError(
            "Gamepad Mode is not yet supported in StretchMujocoDriver."
        )

        def code_to_run():
            try:
                self.sim.stop_trajectory()
            except NotImplementedError as e:
                return False, str(e)
            self.gamepad_teleop.do_double_beep(self.robot)
            self.sim.base.pull_status()

        self.change_mode("gamepad", code_to_run)
        return True, "Now in gamepad mode."

    def activate_streaming_position(self, request):
        self.streaming_position_activated = True
        self.get_logger().info("Activated streaming position.")
        return True, "Activated streaming position."

    def deactivate_streaming_position(self, request):
        self.streaming_position_activated = False
        self.get_logger().info("Deactivated streaming position.")
        return True, "Deactivated streaming position."

    # SERVICE CALLBACKS ##############

    def stop_the_robot_callback(self, request, response):
        with self.robot_stop_lock:
            self.sim.move_by(Actuators.base_translate, 0.0)
            self.sim.move_by(Actuators.base_rotate, 0.0)
            self.sim.move_by(Actuators.arm, 0.0)
            self.sim.move_by(Actuators.lift, 0.0)

            self.sim.move_by("head_pan", 0.0)
            self.sim.move_by("head_tilt", 0.0)
            self.sim.move_by("wrist_yaw", 0.0)
            self.sim.move_by("gripper", 0.0)

        self.get_logger().info(
            "Received stop_the_robot service call, so commanded all actuators to stop."
        )
        response.success = True
        response.message = "Stopped the robot."
        return response

    def home_the_robot_callback(self, request, response):
        self.get_logger().info("Received home_the_robot service call.")
        success, message = self.home_the_robot()
        response.success = success
        response.message = message
        return response

    def stow_the_robot_callback(self, request, response):
        self.get_logger().info("Received stow_the_robot service call.")
        success, message = self.stow_the_robot()
        response.success = success
        response.message = message
        return response

    def navigation_mode_service_callback(self, request, response):
        success, message = self.turn_on_navigation_mode()
        response.success = success
        response.message = message
        return response

    def position_mode_service_callback(self, request, response):
        success, message = self.turn_on_position_mode()
        response.success = success
        response.message = message
        return response

    def trajectory_mode_service_callback(self, request, response):
        success, message = self.turn_on_trajectory_mode()
        response.success = success
        response.message = message
        return response

    def gamepad_mode_service_callback(self, request, response):
        success, message = self.turn_on_gamepad_mode()
        response.success = success
        response.message = message
        return response

    def runstop_service_callback(self, request, response):
        self.get_logger().info("Received runstop_the_robot service call.")
        self.runstop_the_robot(request.data)
        response.success = True
        response.message = f"is_runstopped: {request.data}"
        return response

    def activate_streaming_position_service_callback(self, request, response):
        success, message = self.activate_streaming_position(request)
        response.success = success
        response.message = message
        return response

    def deactivate_streaming_position_service_callback(self, request, response):
        success, message = self.deactivate_streaming_position(request)
        response.success = success
        response.message = message
        return response

    def get_joint_states_callback(self, request, response):
        joint_limits = JointState()
        joint_limits.header.stamp = self.get_clock().now().to_msg()

        joint_limits_from_sim = self.sim.pull_joint_limits()
        for actuator, min_max in joint_limits_from_sim.items():
            joint_name = actuator.get_joint_names_in_mjcf()[0]
            min_limit, max_limit = min_max
            if actuator == Actuators.arm:
                joint_name = "joint_arm"  # Instead of the telescoping names
                max_limit *= 4  # 4x the telescoping limit
            if actuator == Actuators.gripper:
                joint_name = "gripper_aperture"  # A different mapping from stretch_core command_groups
            if actuator in [
                Actuators.gripper_left_finger,
                Actuators.gripper_right_finger,
            ]:
                joint_name = joint_name.replace(
                    "_open", ""
                )  # A different mapping from stretch_core command_groups

            joint_limits.name.append(joint_name)  # type:ignore
            joint_limits.position.append(min_limit)
            joint_limits.velocity.append(max_limit)

        # add "wrist_extension" because it's expected downstream
        arm_joint_limit = joint_limits_from_sim[Actuators.arm]
        joint_limits.name.append("wrist_extension")  # type:ignore
        joint_limits.position.append(arm_joint_limit[0])
        joint_limits.velocity.append(arm_joint_limit[1] * 4)  # 4x the telescoping limit

        self.joint_limits_pub.publish(joint_limits)
        response.success = True
        response.message = ""
        return response

    def self_collision_avoidance_callback(self, request, response):
        # enable_self_collision_avoidance = request.data
        # if enable_self_collision_avoidance:
        #     self.sim.enable_collision_mgmt()
        # else:
        #     self.sim.disable_collision_mgmt()

        response.success = False
        response.message = "collision avoidance is not supported in simulation mode."
        # response.message = (
        #     f"is self collision avoidance enabled: {enable_self_collision_avoidance}"
        # )
        return response

    def parameter_callback(self, parameters: list[Parameter]) -> SetParametersResult:
        """
        Update the parameters that allow for dynamic updates.
        """
        for parameter in parameters:
            if parameter.name == "default_goal_timeout_s":
                self.default_goal_timeout_s = parameter.value or DEFAULT_GOAL_TIMEOUT
                self.default_goal_timeout_duration = Duration(
                    seconds=self.default_goal_timeout_s  # type: ignore
                )
                self.get_logger().info(
                    f"Set default_goal_timeout_s to {self.default_goal_timeout_s}"
                )
        return SetParametersResult(successful=True)

    def home_the_robot(self):
        self.robot_mode_rwlock.acquire_read()
        can_home = self.robot_mode in self.control_modes
        last_robot_mode = copy.copy(self.robot_mode)
        self.robot_mode_rwlock.release_read()
        if not can_home:
            errmsg = f"Cannot home while in mode={last_robot_mode}."
            self.get_logger().error(errmsg)
            return False, errmsg
        self.change_mode("homing", lambda: None)
        self.sim.home()
        self.change_mode(last_robot_mode, lambda: None)
        return True, "Homed."

    def stow_the_robot(self):
        self.robot_mode_rwlock.acquire_read()
        can_stow = self.robot_mode in self.control_modes
        last_robot_mode = copy.copy(self.robot_mode)
        self.robot_mode_rwlock.release_read()
        if not can_stow:
            errmsg = f"Cannot stow while in mode={last_robot_mode}."
            self.get_logger().error(errmsg)
            return False, errmsg
        self.change_mode("stowing", lambda: None)
        self.sim.stow()
        self.change_mode(last_robot_mode, lambda: None)
        return True, "Stowed."

    def is_runstopped(self):
        return self.robot_mode == "runstopped"

    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            self.robot_mode_rwlock.acquire_read()
            already_runstopped = self.robot_mode == "runstopped"
            if not already_runstopped:
                self.prerunstop_mode = copy.copy(self.robot_mode)
            self.robot_mode_rwlock.release_read()
            if already_runstopped:
                return
            self.change_mode("runstopped", lambda: None)
        else:
            self.robot_mode_rwlock.acquire_read()
            already_not_runstopped = self.robot_mode != "runstopped"
            self.robot_mode_rwlock.release_read()
            if already_not_runstopped:
                return
            self.change_mode(self.prerunstop_mode, lambda: None)

    # ROS Setup #################
    def ros_setup(self):
        self.node_name = self.get_name()

        self.get_logger().info(
            "For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc."
        )

        self.get_logger().info("{0} started".format(self.node_name))

        # Handle the non_dxl status in local loop, not thread
        # if not self.sim.startup(start_non_dxl_thread=False,
        #                           start_dxl_thread=True,
        #                           start_sys_mon_thread=True):
        #     self.get_logger().fatal('Robot startup failed.')
        #     rclpy.shutdown()
        #     exit()
        # if not self.sim.is_homed():
        #     self.get_logger().warn("Robot not homed. Call /home_the_robot service.")

        # Create Gamepad Teleop instance
        # self.gamepad_teleop = gamepad_teleop.GamePadTeleop(robot_instance=False,print_dongle_status=False, lock=self.robot_stop_lock)
        # self.gamepad_teleop.startup(self.robot)

        self.declare_parameter("mode", "position")
        mode = self.get_parameter("mode").value
        if mode not in self.control_modes:
            self.get_logger().warn(
                f"{self.node_name} given invalid mode={mode}, using position instead"
            )
            mode = "position"

        self.declare_parameter("broadcast_odom_tf", False)
        self.broadcast_odom_tf = self.get_parameter("broadcast_odom_tf").value
        self.get_logger().info("broadcast_odom_tf = " + str(self.broadcast_odom_tf))
        if self.broadcast_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
            self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        stretch_core_path = get_package_share_path("stretch_core")
        self.declare_parameter(
            "controller_calibration_file",
            str(stretch_core_path / "config" / "controller_calibration_head.yaml"),
        )
        # large_ang = np.radians(45.0)
        # filename = self.get_parameter('controller_calibration_file').value
        # self.get_logger().debug('Loading controller calibration parameters for the head from YAML file named {0}'.format(filename))
        # with open(filename, 'r') as fid:
        #     self.controller_parameters = yaml.safe_load(fid)

        #     self.get_logger().debug('controller parameters loaded = {0}'.format(self.controller_parameters))

        #     self.head_tilt_calibrated_offset_rad = self.controller_parameters['tilt_angle_offset']
        #     ang = self.head_tilt_calibrated_offset_rad
        #     if (abs(ang) > large_ang):
        #         self.get_logger().warn('self.head_tilt_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
        #     self.get_logger().debug('self.head_tilt_calibrated_offset_rad in degrees ='
        #                            ' {0}'.format(np.degrees(self.head_tilt_calibrated_offset_rad)))

        #     self.head_pan_calibrated_offset_rad = self.controller_parameters['pan_angle_offset']
        #     ang = self.head_pan_calibrated_offset_rad
        #     if (abs(ang) > large_ang):
        #         self.get_logger().warn('self.head_pan_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
        #     self.get_logger().debug('self.head_pan_calibrated_offset_rad in degrees ='
        #                            ' {0}'.format(np.degrees(self.head_pan_calibrated_offset_rad)))

        #     self.head_pan_calibrated_looked_left_offset_rad = self.controller_parameters['pan_looked_left_offset']
        #     ang = self.head_pan_calibrated_looked_left_offset_rad
        #     if (abs(ang) > large_ang):
        #         self.get_logger().warn('self.head_pan_calibrated_looked_left_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
        #     self.get_logger().debug(
        #         'self.head_pan_calibrated_looked_left_offset_rad in degrees = {0}'.format(
        #             np.degrees(self.head_pan_calibrated_looked_left_offset_rad)))

        #     self.head_tilt_backlash_transition_angle_rad = self.controller_parameters['tilt_angle_backlash_transition']
        #     self.get_logger().debug(
        #         'self.head_tilt_backlash_transition_angle_rad in degrees = {0}'.format(
        #             np.degrees(self.head_tilt_backlash_transition_angle_rad)))

        #     self.head_tilt_calibrated_looking_up_offset_rad = self.controller_parameters['tilt_looking_up_offset']
        #     ang = self.head_tilt_calibrated_looking_up_offset_rad
        #     if (abs(ang) > large_ang):
        #         self.get_logger().warn('self.head_tilt_calibrated_looking_up_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
        #     self.get_logger().debug(
        #         'self.head_tilt_calibrated_looking_up_offset_rad in degrees = {0}'.format(
        #             np.degrees(self.head_tilt_calibrated_looking_up_offset_rad)))

        #     self.wrist_extension_calibrated_retracted_offset_m = self.controller_parameters['arm_retracted_offset']
        #     m = self.wrist_extension_calibrated_retracted_offset_m
        #     if (abs(m) > 0.05):
        #         self.get_logger().warn('self.wrist_extension_calibrated_retracted_offset_m HAS AN UNUSUALLY LARGE MAGNITUDE')
        #     self.get_logger().debug(
        #         'self.wrist_extension_calibrated_retracted_offset_m in meters = {0}'.format(
        #             self.wrist_extension_calibrated_retracted_offset_m))

        self.linear_velocity_mps = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        self.angular_velocity_radps = 0.0  # rad/s ROS SI standard for cmd_vel (REP 103)

        self.max_arm_height = 1.1

        self.odom_pub = self.create_publisher(Odometry, "odom", 1)
        self.laser_scan_pub = self.create_publisher(
            LaserScan,
            "/scan_filtered",
            qos_profile=QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self.camera_publishers = {
            camera.name: self.create_publisher(
                Image,
                get_camera_topic_name(camera),
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
        }
        self.camera_compressed_publishers = {
            camera.name: self.create_publisher(
                CompressedImage,
                f"{get_camera_topic_name(camera)}/compressed",
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
        }
        self.pointcloud_publishers = {
            camera.name: self.create_publisher(
                PointCloud2,
                get_camera_pointcloud_topic_name(camera),
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
            if camera.is_depth
        }
        self.camera_info_publishers = {
            camera.name: self.create_publisher(
                CameraInfo,
                get_camera_info_topic_name(camera),
                qos_profile=QoSProfile(
                    depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
                ),
            )
            for camera in self.sim._cameras_to_use
        }

        self.clock_pub = self.create_publisher(
            msg_type=Clock, topic="/clock", qos_profile=5
        )

        self.power_pub = self.create_publisher(BatteryState, "battery", 1)
        self.homed_pub = self.create_publisher(Bool, "is_homed", 1)
        self.mode_pub = self.create_publisher(String, "mode", 1)
        self.tool_pub = self.create_publisher(String, "tool", 1)
        self.streaming_position_mode_pub = self.create_publisher(
            Bool, "is_streaming_position", 1
        )

        self.imu_mobile_base_pub = self.create_publisher(Imu, "imu_mobile_base", 1)
        self.magnetometer_mobile_base_pub = self.create_publisher(
            MagneticField, "magnetometer_mobile_base", 1
        )
        self.imu_wrist_pub = self.create_publisher(Imu, "imu_wrist", 1)
        self.runstop_event_pub = self.create_publisher(Bool, "is_runstopped", 1)

        self.is_gamepad_dongle_pub = self.create_publisher(Bool, "is_gamepad_dongle", 1)
        self.gamepad_state_pub = self.create_publisher(
            Joy, "stretch_gamepad_state", 1
        )  # decode using gamepad_conversion.unpack_joy_to_gamepad_state() on client side

        self.main_group = ReentrantCallbackGroup()
        self.mutex_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Twist,
            "cmd_vel",
            self.set_mobile_base_velocity_callback,
            1,
            callback_group=self.main_group,
        )

        self.create_subscription(
            Joy,
            "gamepad_joy",
            self.set_gamepad_motion_callback,
            1,
            callback_group=self.main_group,
        )

        self.create_subscription(
            Float64MultiArray,
            "joint_pose_cmd",
            self.set_robot_streaming_position_callback,
            1,
            callback_group=self.main_group,
        )

        self.declare_parameter("rate", DEFAULT_JOINT_STATE_HZ)
        self.joint_state_rate: float = (
            self.get_parameter("rate").value or DEFAULT_JOINT_STATE_HZ
        )

        self.declare_parameter(
            "timeout",
            DEFAULT_TIMEOUT,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Timeout (sec) after which Twist/Joy commands are considered stale",
            ),
        )
        self.timeout_s = self.get_parameter("timeout").value or DEFAULT_TIMEOUT
        self.timeout = Duration(seconds=self.timeout_s)  # type: ignore
        self.declare_parameter(
            "default_goal_timeout_s",
            DEFAULT_GOAL_TIMEOUT,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Default timeout (sec) for goal execution",
            ),
        )
        self.default_goal_timeout_s: float = (
            self.get_parameter("default_goal_timeout_s").value or DEFAULT_GOAL_TIMEOUT
        )
        self.default_goal_timeout_duration = Duration(
            seconds=self.default_goal_timeout_s  # type: ignore
        )
        self.get_logger().info(f"rate = {self.joint_state_rate} Hz")
        self.get_logger().info(f"twist timeout = {self.timeout_s} s")

        self.base_frame_id = "base_link"
        self.get_logger().info(f"base_frame_id = {self.base_frame_id}")
        self.odom_frame_id = "odom"
        self.get_logger().info(f"odom_frame_id = {self.odom_frame_id}")

        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 1)
        self.joint_limits_pub = self.create_publisher(JointState, "joint_limits", 1)

        self.last_twist_time = self.get_clock().now()
        self.last_gamepad_joy_time = self.get_clock().now()

        # Add a callback for updating parameters
        self.add_on_set_parameters_callback(self.parameter_callback)

        # self.diagnostics = StretchDiagnostics(self, self.robot)

        self.switch_to_navigation_mode_service = self.create_service(
            Trigger,
            "/switch_to_navigation_mode",
            self.navigation_mode_service_callback,
            callback_group=self.main_group,
        )

        self.switch_to_position_mode_service = self.create_service(
            Trigger,
            "/switch_to_position_mode",
            self.position_mode_service_callback,
            callback_group=self.main_group,
        )

        self.switch_to_trajectory_mode_service = self.create_service(
            Trigger,
            "/switch_to_trajectory_mode",
            self.trajectory_mode_service_callback,
            callback_group=self.main_group,
        )

        self.switch_to_gamepad_mode_service = self.create_service(
            Trigger,
            "/switch_to_gamepad_mode",
            self.gamepad_mode_service_callback,
            callback_group=self.main_group,
        )

        self.activate_streaming_position_service = self.create_service(
            Trigger,
            "/activate_streaming_position",
            self.activate_streaming_position_service_callback,
            callback_group=self.main_group,
        )

        self.deactivate_streaming_position_service = self.create_service(
            Trigger,
            "/deactivate_streaming_position",
            self.deactivate_streaming_position_service_callback,
            callback_group=self.main_group,
        )

        self.stop_the_robot_service = self.create_service(
            Trigger,
            "/stop_the_robot",
            self.stop_the_robot_callback,
            callback_group=self.main_group,
        )

        self.home_the_robot_service = self.create_service(
            Trigger,
            "/home_the_robot",
            self.home_the_robot_callback,
            callback_group=self.main_group,
        )

        self.stow_the_robot_service = self.create_service(
            Trigger,
            "/stow_the_robot",
            self.stow_the_robot_callback,
            callback_group=self.main_group,
        )

        self.runstop_service = self.create_service(
            SetBool,
            "/runstop",
            self.runstop_service_callback,
            callback_group=self.main_group,
        )

        self.get_joint_states = self.create_service(
            Trigger,
            "/get_joint_states",
            self.get_joint_states_callback,
            callback_group=self.main_group,
        )

        self.self_collision_avoidance = self.create_service(
            SetBool,
            "/self_collision_avoidance",
            self.self_collision_avoidance_callback,
            callback_group=self.main_group,
        )

        # start action server for joint trajectories
        self.declare_parameter("fail_out_of_range_goal", False)
        self.fail_out_of_range_goal = bool(
            self.get_parameter("fail_out_of_range_goal").value
        )

        self.declare_parameter(
            "fail_if_motor_initial_point_is_not_trajectory_first_point", True
        )
        self.fail_if_motor_initial_point_is_not_trajectory_first_point: bool = bool(
            self.get_parameter(
                "fail_if_motor_initial_point_is_not_trajectory_first_point"
            ).value
        )

        self.declare_parameter("action_server_rate", DEFAULT_ACTION_SERVER_HZ)
        self.action_server_rate: float = (
            self.get_parameter("action_server_rate").value or DEFAULT_ACTION_SERVER_HZ
        )

        self.joint_trajectory_action = JointTrajectoryAction(
            self, self.action_server_rate
        )

        # Switch to mode:
        self.get_logger().debug("mode = " + str(mode))
        if mode == "position":
            self.turn_on_position_mode()
        elif mode == "navigation":
            self.turn_on_navigation_mode()
        elif mode == "trajectory":
            self.turn_on_trajectory_mode()
        elif mode == "gamepad":
            self.turn_on_gamepad_mode()

        # start loop to command the mobile base velocity, publish
        # odometry, and publish joint states
        timer_period: float = 1.0 / self.joint_state_rate
        self.timer = self.create_timer(
            timer_period,
            self.command_mobile_base_velocity_and_publish_state,
            callback_group=self.mutex_group,
        )

        # self.create_timer(
        #     1/15,
        #     self.publish_camera_and_lidar,
        # )


def create_laser_scan_msg(lidar_data: np.ndarray, timestamp: TimeMsg, frame_id: str):
    ranges = lidar_data.tolist()

    laser_scan_msg = (
        LaserScan()
    )  # https://docs.ros.org/en/humble/p/sensor_msgs/msg/LaserScan.html
    laser_scan_msg.header = Header()
    laser_scan_msg.header.stamp = timestamp
    laser_scan_msg.header.frame_id = frame_id
    laser_scan_msg.angle_min = 0.0
    laser_scan_msg.angle_max = np.pi * 2
    laser_scan_msg.angle_increment = laser_scan_msg.angle_max / len(ranges)
    laser_scan_msg.range_min = 0.2
    laser_scan_msg.range_max = 20.0
    laser_scan_msg.ranges = ranges

    return laser_scan_msg


def create_pointcloud_msg(camera_info_msg: CameraInfo, depth_image):
    fx = camera_info_msg.k[0]
    fy = camera_info_msg.k[4]
    cx = camera_info_msg.k[2]
    cy = camera_info_msg.k[5]

    height, width = depth_image.shape
    xx, yy = np.meshgrid(np.arange(width), np.arange(height))
    valid = (depth_image > 0) & np.isfinite(depth_image)

    z = depth_image[valid]
    x = (xx[valid] - cx) * z / fx
    y = (yy[valid] - cy) * z / fy

    points = np.stack((x, y, z), axis=-1)

    cloud_msg = pc2.create_cloud_xyz32(camera_info_msg.header, points)
    return cloud_msg


def create_pointcloud_rgb_msg(
    camera_info_msg: CameraInfo, rgb_image: np.ndarray, depth_image: np.ndarray
):
    fx = camera_info_msg.k[0]
    fy = camera_info_msg.k[4]
    cx = camera_info_msg.k[2]
    cy = camera_info_msg.k[5]

    height, width = depth_image.shape

    xx, yy = np.meshgrid(np.arange(width), np.arange(height))
    valid = (depth_image > 0) & np.isfinite(depth_image)

    z = depth_image[valid]
    x = (xx[valid] - cx) * z / fx
    y = (yy[valid] - cy) * z / fy

    r = rgb_image[:, :, 2][valid]
    g = rgb_image[:, :, 1][valid]
    b = rgb_image[:, :, 0][valid]
    rgb = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)

    # Create XYZRGB tuples
    cloud_data = [(x[i], y[i], z[i], rgb[i]) for i in range(len(z))]

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    header = Header()
    header.stamp = camera_info_msg.header.stamp
    header.frame_id = (
        camera_info_msg.header.frame_id
    )  # typically "camera_link" or similar

    cloud_msg = pc2.create_cloud(header, fields, cloud_data)
    return cloud_msg


__COMPRESSED_DEPTH_16UC1_HEADER = array.array("B", [0] * 12)


def compress_depth_image(frame: np.ndarray):
    """
    Converts a F32 depth map in meters to a U16 map in millimeters
    """
    normalized_array = (frame * 1000).astype(np.uint16)

    _, encoded_image = cv2.imencode(".png", normalized_array)

    ros_image_compressed = CompressedImage()
    ros_image_compressed.format = "16uc1; compressedDepth"
    ros_image_compressed.data = __COMPRESSED_DEPTH_16UC1_HEADER + array.array(
        "B", encoded_image.tobytes()
    )

    return ros_image_compressed


def create_camera_info(
    camera_settings: CameraSettings, frame_id: str, timestamp: TimeMsg
):
    camera_info_msg = CameraInfo()
    camera_info_msg.header = Header()
    camera_info_msg.header.stamp = timestamp
    camera_info_msg.header.frame_id = frame_id
    camera_info_msg.width = camera_settings.width
    camera_info_msg.height = camera_settings.height
    camera_info_msg.distortion_model = "plumb_bob"

    camera_info_msg.d = camera_settings.get_distortion_params_d()
    camera_info_msg.k = camera_settings.get_intrinsic_params_k()
    camera_info_msg.p = camera_settings.get_projection_matrix_p()

    if camera_settings.crop is not None:
        camera_info_msg.roi.x_offset = camera_settings.crop.x_offset
        camera_info_msg.roi.y_offset = camera_settings.crop.y_offset
        camera_info_msg.roi.width = camera_settings.crop.width
        camera_info_msg.roi.height = camera_settings.crop.height

    return camera_info_msg


@cache
def get_camera_topic_name(camera: StretchCameras):
    """
    Topic names to match the camera topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_d405_rgb:
        return "/gripper_camera/image_raw"
    if camera == StretchCameras.cam_d405_depth:
        return "/gripper_camera/depth/image_rect_raw"
    if camera == StretchCameras.cam_d435i_rgb:
        return "/camera/color/image_raw"
    if camera == StretchCameras.cam_d435i_depth:
        return "/camera/depth/image_rect_raw"
    if camera == StretchCameras.cam_nav_rgb:
        return "/navigation_camera/image_raw"

    raise NotImplementedError(f"Camera {camera} topic mapping is not implemented")


@cache
def get_camera_info_topic_name(camera: StretchCameras):
    """
    Topic names to match the camera_info topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_d405_rgb:
        return "/gripper_camera/camera_info"
    if camera == StretchCameras.cam_d405_depth:
        return "/gripper_camera/depth/camera_info"
    if camera == StretchCameras.cam_d435i_rgb:
        return "/camera/color/camera_info"
    if camera == StretchCameras.cam_d435i_depth:
        return "/camera/depth/camera_info"
    if camera == StretchCameras.cam_nav_rgb:
        return "/navigation_camera/camera_info"

    raise NotImplementedError(f"Camera {camera} topic mapping is not implemented")


@cache
def get_camera_pointcloud_topic_name(camera: StretchCameras):
    """
    Topic names to match the pointcloud2 topics published by the real Stretch robot.
    """
    if camera == StretchCameras.cam_d405_rgb:
        raise KeyError(f"{camera} camera does not have a pointcloud.")
    if camera == StretchCameras.cam_d405_depth:
        return "/gripper_camera/depth/color/points"
    if camera == StretchCameras.cam_d435i_rgb:
        raise KeyError(f"{camera} camera does not have a pointcloud.")
    if camera == StretchCameras.cam_d435i_depth:
        return "/camera/depth/color/points"
    if camera == StretchCameras.cam_nav_rgb:
        raise KeyError(f"{camera} camera does not have a pointcloud.")

    raise NotImplementedError(f"Camera {camera} topic mapping is not implemented")


@cache
def get_camera_frame(camera: StretchCameras):
    """
    Matches the simulation camera with the optical frame on the robot urdf.
    """
    if camera == StretchCameras.cam_d405_rgb:
        return "gripper_camera_color_optical_frame"
    if camera == StretchCameras.cam_d405_depth:
        return "gripper_camera_depth_optical_frame"
    if camera == StretchCameras.cam_d435i_rgb:
        return "camera_color_optical_frame"
    if camera == StretchCameras.cam_d435i_depth:
        return "camera_depth_optical_frame"
    if camera == StretchCameras.cam_nav_rgb:
        return "link_head_nav_cam"

    raise NotImplementedError(f"Camera {camera} frame is not implemented")


def main():
    rclpy.init()

    node = StretchMujocoDriver()

    try:
        while rclpy.ok() and node.sim.is_running():
            rclpy.spin_once(node)

    except KeyboardInterrupt:
        print("Detecting KeyboardInterrupt")
    finally:
        print("Stopping Stretch Mujoco Driver")
        node.sim.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
