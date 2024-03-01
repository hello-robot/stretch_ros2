#! /usr/bin/env python3

import copy
import yaml
import numpy as np
import threading
from .rwlock import RWLock
import stretch_body.robot as rb
import stretch_body
from stretch_body.hello_utils import ThreadServiceExit

import tf2_ros
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState, Imu, MagneticField
from std_msgs.msg import Bool, String

from hello_helpers.gripper_conversion import GripperConversion
from .joint_trajectory_server import JointTrajectoryAction
from .stretch_diagnostics import StretchDiagnostics

GRIPPER_DEBUG = False
BACKLASH_DEBUG = False


class StretchDriver(Node):

    def __init__(self):
        super().__init__('stretch_driver')
        self.use_robotis_head = True
        self.use_robotis_end_of_arm = True

        self.default_goal_timeout_s = 10.0
        self.default_goal_timeout_duration = Duration(seconds=self.default_goal_timeout_s)

        # Initialize calibration offsets
        self.head_tilt_calibrated_offset_rad = 0.0
        self.head_pan_calibrated_offset_rad = 0.0

        # Initialize backlash state
        self.backlash_state = {'head_tilt_looking_up': False,
                               'head_pan_looked_left': False,
                               'wrist_extension_retracted': False}

        # Initialize backlash offsets
        self.head_pan_calibrated_looked_left_offset_rad = 0.0
        self.head_tilt_calibrated_looking_up_offset_rad = 0.0
        self.wrist_extension_calibrated_retracted_offset_m = 0.0
        self.head_tilt_backlash_transition_angle_rad = -0.4

        self.gripper_conversion = GripperConversion()

        self.robot_stop_lock = threading.Lock()

        self.robot_mode_rwlock = RWLock()
        self.robot_mode = None

        self.control_modes = ['position', 'navigation', 'trajectory']
        self.prev_runstop_state = None
        self.dirty_command = False

        self.voltage_history = []
        self.charging_state_history = [BatteryState.POWER_SUPPLY_STATUS_UNKNOWN] * 10
        self.charging_state = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN

        self.ros_setup()

    # MOBILE BASE VELOCITY METHODS ############

    def set_mobile_base_velocity_callback(self, twist):
        self.robot_mode_rwlock.acquire_read()
        if self.robot_mode != 'navigation':
            self.get_logger().error('{0} action server must be in navigation mode to '
                                    'receive a twist on cmd_vel. '
                                    'Current mode = {1}.'.format(self.node_name, self.robot_mode))
            self.robot_mode_rwlock.release_read()
            return
        self.linear_velocity_mps = twist.linear.x
        self.angular_velocity_radps = twist.angular.z
        self.last_twist_time = self.get_clock().now()
        self.robot_mode_rwlock.release_read()

    def command_mobile_base_velocity_and_publish_state(self):
        self.robot_mode_rwlock.acquire_read()

        if BACKLASH_DEBUG:
            print('***')
            print('self.backlash_state =', self.backlash_state)

        # set new mobile base velocities, if appropriate
        # check on thread safety for this with callback that sets velocity command values
        if self.robot_mode == 'navigation':
            time_since_last_twist = self.get_clock().now() - self.last_twist_time
            if time_since_last_twist < self.timeout:
                self.robot.base.set_velocity(self.linear_velocity_mps, self.angular_velocity_radps)
                # self.robot.push_command() #Moved to main
            else:
                # Too much information in general, although it could be blocked, since it's just INFO.
                self.robot.base.set_velocity(0.0, 0.0)
                # self.robot.push_command() #Moved to main

        # get copy of the current robot status (uses lock held by the robot)
        robot_status = self.robot.get_status()

        # In the future, consider using time stamps from the robot's
        # motor control boards and other boards. These would need to
        # be synchronized with the ros clock.
        # robot_time = robot_status['timestamp_pc']
        # self.get_logger().info('robot_time =', robot_time)
        # current_time = rospy.Time.from_sec(robot_time)

        current_clock = self.get_clock().now()
        current_time = current_clock.to_msg()

        # obtain odometry
        # assign relevant base status to variables
        base_status = robot_status['base']
        x = base_status['x']
        y = base_status['y']
        theta = base_status['theta']
        x_vel = base_status['x_vel']
        y_vel = base_status['y_vel']
        theta_vel = base_status['theta_vel']

        # assign relevant arm status to variables
        arm_status = robot_status['arm']
        if self.backlash_state['wrist_extension_retracted']:
            arm_backlash_correction = self.wrist_extension_calibrated_retracted_offset_m
        else:
            arm_backlash_correction = 0.0

        if BACKLASH_DEBUG:
            print('arm_backlash_correction =', arm_backlash_correction)
        pos_out = arm_status['pos'] + arm_backlash_correction
        vel_out = arm_status['vel']
        eff_out = arm_status['motor']['effort_pct']

        lift_status = robot_status['lift']
        pos_up = lift_status['pos']
        vel_up = lift_status['vel']
        eff_up = lift_status['motor']['effort_pct']

        if self.use_robotis_end_of_arm:
            # assign relevant wrist status to variables
            wrist_yaw_status = robot_status['end_of_arm']['wrist_yaw']
            wrist_yaw_rad = wrist_yaw_status['pos']
            wrist_yaw_vel = wrist_yaw_status['vel']
            wrist_yaw_effort = wrist_yaw_status['effort']
            
            dex_wrist_attached = False
            if 'wrist_pitch' in robot_status['end_of_arm']:
                dex_wrist_attached = True
            
            if dex_wrist_attached:
                wrist_pitch_status = robot_status['end_of_arm']['wrist_pitch']
                wrist_pitch_rad = wrist_pitch_status['pos']
                wrist_pitch_vel = wrist_pitch_status['vel']
                wrist_pitch_effort = wrist_pitch_status['effort']

                wrist_roll_status = robot_status['end_of_arm']['wrist_roll']
                wrist_roll_rad = wrist_roll_status['pos']
                wrist_roll_vel = wrist_roll_status['vel']
                wrist_roll_effort = wrist_roll_status['effort']

            # assign relevant gripper status to variables
            gripper_status = robot_status['end_of_arm']['stretch_gripper']
            if GRIPPER_DEBUG:
                print('-----------------------')
                print('gripper_status[\'pos\'] =', gripper_status['pos'])
                print('gripper_status[\'pos_pct\'] =', gripper_status['pos_pct'])
            gripper_aperture_m, gripper_finger_rad, gripper_finger_effort, gripper_finger_vel = \
                self.gripper_conversion.status_to_all(gripper_status)
            if GRIPPER_DEBUG:
                print('gripper_aperture_m =', gripper_aperture_m)
                print('gripper_finger_rad =', gripper_finger_rad)
                print('-----------------------')

        if self.use_robotis_head:
            # assign relevant head pan status to variables
            head_pan_status = robot_status['head']['head_pan']
            if self.backlash_state['head_pan_looked_left']:
                pan_backlash_correction = self.head_pan_calibrated_looked_left_offset_rad
            else:
                pan_backlash_correction = 0.0
            if BACKLASH_DEBUG:
                print('pan_backlash_correction =', pan_backlash_correction)
            head_pan_rad = head_pan_status['pos'] + self.head_pan_calibrated_offset_rad + pan_backlash_correction
            head_pan_vel = head_pan_status['vel']
            head_pan_effort = head_pan_status['effort']

            # assign relevant head tilt status to variables
            head_tilt_status = robot_status['head']['head_tilt']
            if self.backlash_state['head_tilt_looking_up']:
                tilt_backlash_correction = self.head_tilt_calibrated_looking_up_offset_rad
            else:
                tilt_backlash_correction = 0.0
            if BACKLASH_DEBUG:
                print('tilt_backlash_correction =', tilt_backlash_correction)
            head_tilt_rad = head_tilt_status['pos'] + self.head_tilt_calibrated_offset_rad + tilt_backlash_correction
            head_tilt_vel = head_tilt_status['vel']
            head_tilt_effort = head_tilt_status['effort']

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

            b = TransformStamped()
            b.header.stamp = current_time
            b.header.frame_id = self.base_frame_id
            b.child_frame_id = "base_footprint"
            b.transform.translation.x = 0.0
            b.transform.translation.y = 0.0
            b.transform.translation.z = 0.0
            b.transform.rotation.x = 0.0
            b.transform.rotation.y = 0.0
            b.transform.rotation.z = 0.0
            b.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(b)

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
        # obtain battery state
        pimu_hardware_id = self.robot.pimu.board_info['hardware_id']
        invalid_reading = float('NaN')
        v = float(robot_status['pimu']['voltage'])
        self.voltage_history.append(v)
        if len(self.voltage_history) > 100:
            self.voltage_history.pop(0)
            self.charging_state_history.pop(0)
            if v > np.mean(self.voltage_history) + 3 * np.std(self.voltage_history):
                self.charging_state_history.append(BatteryState.POWER_SUPPLY_STATUS_CHARGING)
            elif v < np.mean(self.voltage_history) - 3 * np.std(self.voltage_history):
                self.charging_state_history.append(BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
            else:
                self.charging_state_history.append(BatteryState.POWER_SUPPLY_STATUS_UNKNOWN)
        filtered_charging_state = max(set(self.charging_state_history), key=self.charging_state_history.count)
        if filtered_charging_state != BatteryState.POWER_SUPPLY_STATUS_UNKNOWN:
            if pimu_hardware_id == 0:
                self.charging_state = filtered_charging_state
            elif pimu_hardware_id == 1:
                if robot_status['pimu']['charger_connected'] == True and filtered_charging_state == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif robot_status['pimu']['charger_connected'] == False and filtered_charging_state == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            elif pimu_hardware_id == 2:
                if robot_status['pimu']['charger_connected'] == True and filtered_charging_state == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_CHARGING
                elif robot_status['pimu']['charger_connected'] == False and filtered_charging_state == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
                    self.charging_state = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        i = float(robot_status['pimu']['current'])
        if self.charging_state == BatteryState.POWER_SUPPLY_STATUS_CHARGING:
            i = float(robot_status['pimu']['current'])
        elif self.charging_state == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING:
            i = -1 * float(robot_status['pimu']['current'])

        # publish battery state
        # TODO: Add way to determine if the robot is charging
        # TODO: Calculate the percentage
        battery_state = BatteryState()
        battery_state.header.stamp = current_time
        battery_state.voltage = v
        battery_state.temperature = invalid_reading
        battery_state.current = i
        battery_state.charge = invalid_reading
        battery_state.capacity = invalid_reading
        battery_state.design_capacity = 18.0
        battery_state.percentage = invalid_reading
        battery_state.power_supply_status = self.charging_state
        battery_state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery_state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        # misuse the 'present' flag to indicated whether the barrel jack button is pressed (i.e. charger is present, but may or may not be providing power)
        if pimu_hardware_id == 0:
            battery_state.present = False
        elif pimu_hardware_id == 1 or pimu_hardware_id == 2:
            battery_state.present = robot_status['pimu']['charger_connected']
        self.power_pub.publish(battery_state)
        
        ##################################################
        # publish homed status
        homed_status = Bool()
        homed_status.data = bool(self.robot.is_homed())
        self.homed_pub.publish(homed_status)

        # publish runstop event
        runstop_event = Bool()
        runstop_event.data = robot_status['pimu']['runstop_event']
        self.runstop_event_pub.publish(runstop_event)

        # publish stretch_driver operation mode
        mode_msg = String()
        mode_msg.data = self.robot_mode
        self.mode_pub.publish(mode_msg)

        # publish end of arm tool
        tool_msg = String()
        tool_msg.data = self.robot.end_of_arm.name
        self.tool_pub.publish(tool_msg)

        # publish joint state for the arm
        joint_state = JointState()
        joint_state.header.stamp = current_time
        # joint_arm_l3 is the most proximal and joint_arm_l0 is the
        # most distal joint of the telescoping arm model. The joints
        # are connected in series such that moving the most proximal
        # joint moves all the other joints in the global frame.
        joint_state.name = ['wrist_extension', 'joint_lift', 'joint_arm_l3', 'joint_arm_l2', 'joint_arm_l1', 'joint_arm_l0']

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

        if self.use_robotis_head:
            head_joint_names = ['joint_head_pan', 'joint_head_tilt']
            joint_state.name.extend(head_joint_names)

            positions.append(head_pan_rad)
            velocities.append(head_pan_vel)
            efforts.append(head_pan_effort)

            positions.append(head_tilt_rad)
            velocities.append(head_tilt_vel)
            efforts.append(head_tilt_effort)

        if self.use_robotis_end_of_arm:
            if dex_wrist_attached:
                end_of_arm_joint_names = ['joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll', 'joint_gripper_finger_left', 'joint_gripper_finger_right']
            else:
                end_of_arm_joint_names = ['joint_wrist_yaw', 'joint_gripper_finger_left', 'joint_gripper_finger_right']
            
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

            positions.append(gripper_finger_rad)
            velocities.append(gripper_finger_vel)
            efforts.append(gripper_finger_effort)

            positions.append(gripper_finger_rad)
            velocities.append(gripper_finger_vel)
            efforts.append(gripper_finger_effort)

        # set joint_state
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = efforts
        self.joint_state_pub.publish(joint_state)

        ##################################################
        # publish IMU sensor data
        imu_status = robot_status['pimu']['imu']
        ax = imu_status['ax']
        ay = imu_status['ay']
        az = imu_status['az']
        gx = imu_status['gx']
        gy = imu_status['gy']
        gz = imu_status['gz']
        qw = imu_status['qw']
        qx = imu_status['qx']
        qy = imu_status['qy']
        qz = imu_status['qz']

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = 'imu_mobile_base'
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
        m.header.frame_id = 'imu_mobile_base'
        self.magnetometer_mobile_base_pub.publish(m)

        accel_status = robot_status['wacc']
        ax = accel_status['ax']
        ay = accel_status['ay']
        az = accel_status['az']

        i = Imu()
        i.header.stamp = current_time
        i.header.frame_id = 'accel_wrist'
        i.linear_acceleration.x = ax
        i.linear_acceleration.y = ay
        i.linear_acceleration.z = az
        self.imu_wrist_pub.publish(i)
        ##################################################

        self.robot_mode_rwlock.release_read()

        # must happen after the read release, otherwise the write lock in change_mode() will cause a deadlock
        if (self.prev_runstop_state == None and runstop_event.data) or (self.prev_runstop_state != None and runstop_event.data != self.prev_runstop_state):
            self.runstop_the_robot(runstop_event.data, just_change_mode=True)
        self.prev_runstop_state = runstop_event.data

        if self.robot.pimu.params.get('ros_fan_on', True):
            self.robot.pimu.set_fan_on()
        self.robot.non_dxl_thread.step()
        if not self.robot_mode == 'trajectory':
            self.robot.push_command() # Main push command
            self.dirty_command = False

    # CHANGE MODES ################

    def change_mode(self, new_mode, code_to_run = None):
        self.robot_mode_rwlock.acquire_write()
        self.robot_mode = new_mode
        
        if code_to_run:
            code_to_run()

        self.get_logger().info(f'Changed to mode = {self.robot_mode}')
        self.robot_mode_rwlock.release_write()

    # TODO : add a freewheel mode or something comparable for the mobile base?

    def turn_on_navigation_mode(self):
        # Navigation mode enables mobile base velocity control via
        # cmd_vel, and disables position-based control of the mobile
        # base.
        def code_to_run():
            self.linear_velocity_mps = 0.0
            self.angular_velocity_radps = 0.0
        self.change_mode('navigation', code_to_run)
        return True, 'Now in navigation mode.'

    def turn_on_position_mode(self):
        # Position mode enables mobile base translation and rotation
        # using position control with sequential incremental rotations
        # and translations. It also disables velocity control of the
        # mobile base. It does not update the virtual prismatic
        # joint. The frames associated with 'floor_link' and
        # 'base_link' become identical in this mode.
        def code_to_run():
            self.robot.base.enable_pos_incr_mode()
        self.change_mode('position', code_to_run)
        return True, 'Now in position mode.'

    def turn_on_trajectory_mode(self):
        # Trajectory mode is able to execute plans from
        # high level planners like MoveIt2. These planners
        # send whole robot waypoint trajectories to the
        # joint trajectory action server, and the underlying
        # Python interface to the robot (Stretch Body) executes
        # the trajectory, respecting each waypoints' time_from_start
        # attribute of the trajectory_msgs/JointTrajectoryPoint
        # message. This allows coordinated motion of the base + arm.
        def code_to_run():
            try:
                self.robot.stop_trajectory()
            except NotImplementedError as e:
                return False, str(e)
            self.robot.base.first_step = True
            self.robot.base.pull_status()

        self.change_mode('trajectory', code_to_run)
        return True, 'Now in trajectory mode.'

    # SERVICE CALLBACKS ##############

    def stop_the_robot_callback(self, request, response):
        with self.robot_stop_lock:
            self.robot.base.translate_by(0.0)
            self.robot.base.rotate_by(0.0)
            self.robot.arm.move_by(0.0)
            self.robot.lift.move_by(0.0)
            # self.robot.push_command() #Moved to main

            self.robot.head.move_by('head_pan', 0.0)
            self.robot.head.move_by('head_tilt', 0.0)
            self.robot.end_of_arm.move_by('wrist_yaw', 0.0)
            self.robot.end_of_arm.move_by('stretch_gripper', 0.0)
            # self.robot.push_command() #Moved to main

        self.get_logger().info('Received stop_the_robot service call, so commanded all actuators to stop.')
        response.success = True
        response.message = 'Stopped the robot.'
        return response

    def home_the_robot_callback(self, request, response):
        success, message = self.home_the_robot()

        self.get_logger().info('Received home_the_robot service call.')
        response.success = success
        response.message = message
        return response

    def stow_the_robot_callback(self, request, response):
        self.stow_the_robot()

        self.get_logger().info('Received stow_the_robot service call.')
        response.success = True
        response.message = 'Stowed.'
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

    def runstop_service_callback(self, request, response):
        self.runstop_the_robot(request.data)

        response.success = True
        response.message = f'is_runstopped: {request.data}'
        return response

    def get_joint_states_callback(self, request, response):
        joint_limits = JointState()
        joint_limits.header.stamp = self.get_clock().now().to_msg()
        cgs = list(set(self.joint_trajectory_action.command_groups) - set([self.joint_trajectory_action.mobile_base_cg, self.joint_trajectory_action.gripper_cg]))
        for cg in cgs:
            lower_limit, upper_limit = cg.range
            joint_limits.name.append(cg.name)
            joint_limits.position.append(lower_limit) # Misuse position array to mean lower limits
            joint_limits.velocity.append(upper_limit) # Misuse velocity array to mean upper limits

        gripper_cg = self.joint_trajectory_action.gripper_cg
        if gripper_cg is not None:
            lower_aperture_limit, upper_aperture_limit = gripper_cg.range_aperture_m
            joint_limits.name.append('gripper_aperture')
            joint_limits.position.append(lower_aperture_limit)
            joint_limits.velocity.append(upper_aperture_limit)

            lower_finger_limit, upper_finger_limit = gripper_cg.range_finger_rad
            joint_limits.name.append('joint_gripper_finger_left')
            joint_limits.position.append(lower_finger_limit)
            joint_limits.velocity.append(upper_finger_limit)
            joint_limits.name.append('joint_gripper_finger_right')
            joint_limits.position.append(lower_finger_limit)
            joint_limits.velocity.append(upper_finger_limit)

        self.joint_limits_pub.publish(joint_limits)
        response.success = True
        response.message = ''
        return response

    def self_collision_avoidance_callback(self, request, response):
        enable_self_collision_avoidance = request.data
        if enable_self_collision_avoidance:
            self.robot.enable_collision_mgmt()
        else:
            self.robot.disable_collision_mgmt()

        response.success = True
        response.message = f'is self collision avoidance enabled: {enable_self_collision_avoidance}'
        return response

    def home_the_robot(self):
        self.robot_mode_rwlock.acquire_read()
        can_home = self.robot_mode in self.control_modes
        last_robot_mode = copy.copy(self.robot_mode)
        self.robot_mode_rwlock.release_read()
        if not can_home:
            errmsg = f'Cannot home while in mode={last_robot_mode}.'
            self.get_logger().error(errmsg)
            return False, errmsg
        def code_to_run():
            pass
        self.change_mode('homing', code_to_run)
        self.robot.home()
        self.change_mode(last_robot_mode, code_to_run)
        return True, 'Homed.'
    
    def stow_the_robot(self):
        self.robot_mode_rwlock.acquire_read()
        can_stow = self.robot_mode in self.control_modes
        last_robot_mode = copy.copy(self.robot_mode)
        self.robot_mode_rwlock.release_read()
        if not can_stow:
            errmsg = f'Cannot stow while in mode={last_robot_mode}.'
            self.get_logger().error(errmsg)
            return False, errmsg
        def code_to_run():
            pass
        self.change_mode('stowing', code_to_run)
        self.robot.stow()
        self.change_mode(last_robot_mode, code_to_run)
        return True, 'Stowed.'

    def runstop_the_robot(self, runstopped, just_change_mode=False):
        if runstopped:
            self.robot_mode_rwlock.acquire_read()
            already_runstopped = self.robot_mode == 'runstopped'
            if not already_runstopped:
                self.prerunstop_mode = copy.copy(self.robot_mode)
            self.robot_mode_rwlock.release_read()
            if already_runstopped:
                return

            def code_to_run():
                pass
            self.change_mode('runstopped', code_to_run)
            if not just_change_mode:
                self.robot.pimu.runstop_event_trigger()
                # self.robot.push_command()#Moved to main
        else:
            self.robot_mode_rwlock.acquire_read()
            already_not_runstopped = self.robot_mode != 'runstopped'
            self.robot_mode_rwlock.release_read()
            if already_not_runstopped:
                return

            def code_to_run():
                pass
            self.change_mode(self.prerunstop_mode, code_to_run)
            if not just_change_mode:
                self.robot.pimu.runstop_event_reset()
                # self.robot.push_command() #Moved to main

    # ROS Setup #################
    def ros_setup(self):
        self.node_name = self.get_name()

        self.get_logger().info("For use with S T R E T C H (TM) RESEARCH EDITION from Hello Robot Inc.")

        self.get_logger().info("{0} started".format(self.node_name))
        
        if int(stretch_body.__version__.split('.')[1]) < 5:
            self.get_logger().fatal("ERROR: Found old stretch_body version. Please upgrade stretch_body to v0.5.0 or above.")
            rclpy.shutdown()
            exit()
        
        self.robot = rb.Robot()
        #Handle the non_dxl status in local loop, not thread
        if not self.robot.startup(start_non_dxl_thread=False,
                                  start_dxl_thread=True,
                                  start_sys_mon_thread=True):
            self.get_logger().fatal('Robot startup failed.')
            rclpy.shutdown()
            exit()
        if not self.robot.is_homed():
            self.get_logger().warn("Robot not homed. Call /home_the_robot service.")

        self.declare_parameter('mode', "position")
        mode = self.get_parameter('mode').value
        if mode not in self.control_modes:
            self.get_logger().warn(f'{self.node_name} given invalid mode={mode}, using position instead')
            mode = 'position'
        self.get_logger().debug('mode = ' + str(mode))
        if mode == "position":
            self.turn_on_position_mode()
        elif mode == "navigation":
            self.turn_on_navigation_mode()
        elif mode == "trajectory":
            self.turn_on_trajectory_mode()

        self.declare_parameter('broadcast_odom_tf', False)
        self.broadcast_odom_tf = self.get_parameter('broadcast_odom_tf').value
        self.get_logger().info('broadcast_odom_tf = ' + str(self.broadcast_odom_tf))
        if self.broadcast_odom_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        large_ang = np.radians(45.0)

        self.declare_parameter('controller_calibration_file', 'NOT SET')
        filename = self.get_parameter('controller_calibration_file').value
        self.get_logger().debug('Loading controller calibration parameters for the head from YAML file named {0}'.format(filename))
        with open(filename, 'r') as fid:
            self.controller_parameters = yaml.safe_load(fid)

            self.get_logger().debug('controller parameters loaded = {0}'.format(self.controller_parameters))

            self.head_tilt_calibrated_offset_rad = self.controller_parameters['tilt_angle_offset']
            ang = self.head_tilt_calibrated_offset_rad
            if (abs(ang) > large_ang):
                self.get_logger().warn('self.head_tilt_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            self.get_logger().debug('self.head_tilt_calibrated_offset_rad in degrees ='
                                   ' {0}'.format(np.degrees(self.head_tilt_calibrated_offset_rad)))

            self.head_pan_calibrated_offset_rad = self.controller_parameters['pan_angle_offset']
            ang = self.head_pan_calibrated_offset_rad
            if (abs(ang) > large_ang):
                self.get_logger().warn('self.head_pan_calibrated_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            self.get_logger().debug('self.head_pan_calibrated_offset_rad in degrees ='
                                   ' {0}'.format(np.degrees(self.head_pan_calibrated_offset_rad)))

            self.head_pan_calibrated_looked_left_offset_rad = self.controller_parameters['pan_looked_left_offset']
            ang = self.head_pan_calibrated_looked_left_offset_rad
            if (abs(ang) > large_ang):
                self.get_logger().warn('self.head_pan_calibrated_looked_left_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            self.get_logger().debug(
                'self.head_pan_calibrated_looked_left_offset_rad in degrees = {0}'.format(
                    np.degrees(self.head_pan_calibrated_looked_left_offset_rad)))

            self.head_tilt_backlash_transition_angle_rad = self.controller_parameters['tilt_angle_backlash_transition']
            self.get_logger().debug(
                'self.head_tilt_backlash_transition_angle_rad in degrees = {0}'.format(
                    np.degrees(self.head_tilt_backlash_transition_angle_rad)))

            self.head_tilt_calibrated_looking_up_offset_rad = self.controller_parameters['tilt_looking_up_offset']
            ang = self.head_tilt_calibrated_looking_up_offset_rad
            if (abs(ang) > large_ang):
                self.get_logger().warn('self.head_tilt_calibrated_looking_up_offset_rad HAS AN UNUSUALLY LARGE MAGNITUDE')
            self.get_logger().debug(
                'self.head_tilt_calibrated_looking_up_offset_rad in degrees = {0}'.format(
                    np.degrees(self.head_tilt_calibrated_looking_up_offset_rad)))

            self.wrist_extension_calibrated_retracted_offset_m = self.controller_parameters['arm_retracted_offset']
            m = self.wrist_extension_calibrated_retracted_offset_m
            if (abs(m) > 0.05):
                self.get_logger().warn('self.wrist_extension_calibrated_retracted_offset_m HAS AN UNUSUALLY LARGE MAGNITUDE')
            self.get_logger().debug(
                'self.wrist_extension_calibrated_retracted_offset_m in meters = {0}'.format(
                    self.wrist_extension_calibrated_retracted_offset_m))

        self.linear_velocity_mps = 0.0  # m/s ROS SI standard for cmd_vel (REP 103)
        self.angular_velocity_radps = 0.0  # rad/s ROS SI standard for cmd_vel (REP 103)

        self.max_arm_height = 1.1

        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)

        self.power_pub = self.create_publisher(BatteryState, 'battery', 1)
        self.homed_pub = self.create_publisher(Bool, 'is_homed', 1)
        self.mode_pub = self.create_publisher(String, 'mode', 1)
        self.tool_pub = self.create_publisher(String, 'tool', 1)

        self.imu_mobile_base_pub = self.create_publisher(Imu, 'imu_mobile_base', 1)
        self.magnetometer_mobile_base_pub = self.create_publisher(MagneticField, 'magnetometer_mobile_base', 1)
        self.imu_wrist_pub = self.create_publisher(Imu, 'imu_wrist', 1)
        self.runstop_event_pub = self.create_publisher(Bool, 'is_runstopped', 1)

        self.group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Twist, "cmd_vel", self.set_mobile_base_velocity_callback, 1, callback_group=self.group)

        self.declare_parameter('rate', 30.0)
        self.joint_state_rate = self.get_parameter('rate').value
        self.declare_parameter('timeout', 0.5)
        self.timeout_s = self.get_parameter('timeout').value
        self.timeout = Duration(seconds=self.timeout_s)
        self.get_logger().info(f"rate = {self.joint_state_rate} Hz")
        self.get_logger().info(f"twist timeout = {self.timeout_s} s")

        self.base_frame_id = 'base_link'
        self.get_logger().info(f"base_frame_id = {self.base_frame_id}")
        self.odom_frame_id = 'odom'
        self.get_logger().info(f"odom_frame_id = {self.odom_frame_id}")

        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.joint_limits_pub = self.create_publisher(JointState, 'joint_limits', 1)

        self.last_twist_time = self.get_clock().now()

        # start action server for joint trajectories
        self.declare_parameter('fail_out_of_range_goal', False)
        self.fail_out_of_range_goal = self.get_parameter('fail_out_of_range_goal').value
        
        self.declare_parameter('action_server_rate', 30.0)
        self.action_server_rate = self.get_parameter('action_server_rate').value

        self.diagnostics = StretchDiagnostics(self, self.robot)

        self.switch_to_navigation_mode_service = self.create_service(Trigger,
                                                                     '/switch_to_navigation_mode',
                                                                     self.navigation_mode_service_callback)

        self.switch_to_position_mode_service = self.create_service(Trigger,
                                                                   '/switch_to_position_mode',
                                                                   self.position_mode_service_callback)

        self.switch_to_trajectory_mode_service = self.create_service(Trigger,
                                                                       '/switch_to_trajectory_mode',
                                                                       self.trajectory_mode_service_callback)

        self.stop_the_robot_service = self.create_service(Trigger,
                                                          '/stop_the_robot',
                                                          self.stop_the_robot_callback)

        self.home_the_robot_service = self.create_service(Trigger,
                                                          '/home_the_robot',
                                                          self.home_the_robot_callback)

        self.stow_the_robot_service = self.create_service(Trigger,
                                                           '/stow_the_robot',
                                                           self.stow_the_robot_callback)

        self.runstop_service = self.create_service(SetBool,
                                                   '/runstop',
                                                   self.runstop_service_callback)

        self.get_joint_states = self.create_service(Trigger,
                                                    '/get_joint_states',
                                                    self.get_joint_states_callback)

        self.self_collision_avoidance = self.create_service(SetBool,
                                                            '/self_collision_avoidance',
                                                            self.self_collision_avoidance_callback)

        # start loop to command the mobile base velocity, publish
        # odometry, and publish joint states
        timer_period = 1.0 / self.joint_state_rate
        self.timer = self.create_timer(timer_period, self.command_mobile_base_velocity_and_publish_state)


def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=2)
        node = StretchDriver()
        node.joint_trajectory_action = JointTrajectoryAction(node, node.action_server_rate)
        executor.add_node(node)
        executor.add_node(node.joint_trajectory_action)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.joint_trajectory_action.destroy_node()
            node.destroy_node()
    except (KeyboardInterrupt, ThreadServiceExit):
        node.robot.stop()


if __name__ == '__main__':
    main()
