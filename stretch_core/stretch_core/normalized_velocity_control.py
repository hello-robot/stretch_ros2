#!/usr/bin/env python3
"""Per-joint normalized ([-1.0, 1.0]) velocity command dispatch, ported from
hello-robot/stretch_visual_servoing's normalized_velocity_control.py.

Unlike the reference implementation, this module has no background thread or
lock of its own: the objects here are meant to be constructed once and then
driven synchronously from stretch_driver's existing periodic control loop and
robot_mode_rwlock, the same way 'navigation' mode already drives
robot.base.set_velocity() directly.

Recognized normalized velocity command keys:
    base_forward, base_counterclockwise, lift_up, arm_out,
    wrist_roll_counterclockwise, wrist_pitch_up, wrist_yaw_counterclockwise,
    head_pan_counterclockwise, head_tilt_up, gripper_open
"""

import time

from stretch_body.hello_utils import map_to_range
from stretch_body.robot_params import RobotParams


def bound_norm_vel(vel):
    return max(-1.0, min(1.0, vel))


class CommandBase:
    def __init__(self):
        self.params = RobotParams().get_params()[1]['base']
        self.dead_zone = 0.0001
        self.max_rotation_vel = 0.5  # rad/s
        self.normal_linear_vel = self.params['motion']['default']['vel_m']
        self.normal_rotation_vel = self.max_rotation_vel * 0.4
        self.acc = self.params['motion']['max']['accel_m']

    def command_stick_to_motion(self, x, y, robot):
        """x: counterclockwise rotation in [-1.0, 1.0], y: forward translation in [-1.0, 1.0]"""
        if abs(x) < self.dead_zone:
            x = 0
        if abs(y) < self.dead_zone:
            y = 0
        v_m, w_r = self._process_stick_to_vel(x, y)
        robot.base.set_velocity(v_m, w_r, a=self.acc)

    def stop_motion(self, robot):
        robot.base.set_velocity(0, 0, a=self.acc)

    def _process_stick_to_vel(self, x, y):
        v_m = map_to_range(abs(y), 0, self.normal_linear_vel)
        if y < 0:
            v_m = -1 * v_m
        x = -1 * x
        w_r = map_to_range(abs(x), 0, self.normal_rotation_vel)
        if x < 0:
            w_r = -1 * w_r
        w_r = -1 * w_r
        return v_m, w_r


class CommandLift:
    def __init__(self):
        self.params = RobotParams().get_params()[1]['lift']
        self.dead_zone = 0.0001
        self.max_linear_vel = self.params['motion']['max']['vel_m']
        self.acc = self.params['motion']['max']['accel_m']

    def command_stick_to_motion(self, x, robot):
        if abs(x) < self.dead_zone:
            x = 0
        v_m = self._process_stick_to_vel(x)
        robot.lift.set_velocity(v_m, a_m=self.acc)

    def stop_motion(self, robot):
        robot.lift.set_velocity(0, a_m=self.acc)

    def _process_stick_to_vel(self, x):
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x < 0:
            v_m = -1 * v_m
        return v_m


class CommandArm:
    def __init__(self):
        self.params = RobotParams().get_params()[1]['arm']
        self.dead_zone = 0.0001
        self.max_linear_vel = self.params['motion']['default']['vel_m']
        self.acc = self.params['motion']['max']['accel_m']

    def command_stick_to_motion(self, x, robot):
        if abs(x) < self.dead_zone:
            x = 0
        v_m = self._process_stick_to_vel(x)
        robot.arm.set_velocity(v_m, a_m=self.acc)

    def stop_motion(self, robot):
        robot.arm.set_velocity(0, a_m=self.acc)

    def _process_stick_to_vel(self, x):
        v_m = map_to_range(abs(x), 0, self.max_linear_vel)
        if x < 0:
            v_m = -1 * v_m
        return v_m


class CommandDxlJoint:
    """Base class for wrist_*/head_*/gripper Dynamixel joint velocity control."""

    def __init__(self, name, max_vel=None, acc_type=None):
        self.params = RobotParams().get_params()[1][name]
        self.name = name
        self.dead_zone = 0.001
        self.max_vel = max_vel if max_vel else self.params['motion']['default']['vel']
        self.acc = self.params['motion'][acc_type]['accel'] if acc_type else None

    def _get_motor(self, robot):
        if 'wrist' in self.name or 'gripper' in self.name:
            return robot.end_of_arm.get_joint(self.name)
        if 'head' in self.name:
            return robot.head.get_joint(self.name)
        raise ValueError('Unrecognized joint group for {0}'.format(self.name))

    def command_stick_to_motion(self, x, robot):
        motor = self._get_motor(robot)
        if abs(x) < self.dead_zone:
            x = 0
        acc = self.params['motion']['max']['accel'] if (x == 0 or 'gripper' in self.name) else self.acc
        v = self._process_stick_to_vel(x)
        motor.set_velocity(v, acc)

    def stop_motion(self, robot):
        motor = self._get_motor(robot)
        motor.set_velocity(0, self.params['motion']['max']['accel'])

    def _process_stick_to_vel(self, x):
        v = map_to_range(abs(x), 0, self.max_vel)
        if x < 0:
            v = -1 * v
        return v


class CommandWristYaw(CommandDxlJoint):
    def __init__(self, name='wrist_yaw', max_vel=1.5, acc_type='slow'):
        super().__init__(name, max_vel, acc_type)


class CommandWristPitch(CommandDxlJoint):
    def __init__(self, name='wrist_pitch', max_vel=1, acc_type='slow'):
        super().__init__(name, max_vel, acc_type)


class CommandWristRoll(CommandDxlJoint):
    def __init__(self, name='wrist_roll', max_vel=None, acc_type='slow'):
        super().__init__(name, max_vel, acc_type)


class CommandHeadPan(CommandDxlJoint):
    def __init__(self, name='head_pan', max_vel=None, acc_type='slow'):
        super().__init__(name, max_vel, acc_type)


class CommandHeadTilt(CommandDxlJoint):
    def __init__(self, name='head_tilt', max_vel=None, acc_type='slow'):
        super().__init__(name, max_vel, acc_type)


class CommandGripper(CommandDxlJoint):
    def __init__(self, name='stretch_gripper', max_vel=None, acc_type='slow'):
        super().__init__(name, max_vel, acc_type)


def create_command_objects():
    """One instance per joint group, meant to be constructed once (e.g. in
    stretch_driver's ros_setup()) and reused across control ticks."""
    return {
        'base': CommandBase(),
        'lift': CommandLift(),
        'arm': CommandArm(),
        'wrist_yaw': CommandWristYaw(),
        'wrist_pitch': CommandWristPitch(),
        'wrist_roll': CommandWristRoll(),
        'head_pan': CommandHeadPan(),
        'head_tilt': CommandHeadTilt(),
        'gripper': CommandGripper(),
    }


def execute_velocity_command(cmd, commands, robot):
    """Dispatch a sparse dict of normalized ([-1.0, 1.0]) joint velocities to
    stretch_body. Does not call robot.push_command() -- that stays centralized
    in stretch_driver's main control loop.

    cmd: dict, only the keys to be commanded need be present (see module docstring)
    commands: dict returned by create_command_objects()
    robot: a stretch_body.robot.Robot instance
    """
    if ('base_forward' in cmd) or ('base_counterclockwise' in cmd):
        vf = bound_norm_vel(cmd.get('base_forward', 0.0))
        vcc = bound_norm_vel(cmd.get('base_counterclockwise', 0.0))
        commands['base'].command_stick_to_motion(vcc, vf, robot)

    if 'lift_up' in cmd:
        commands['lift'].command_stick_to_motion(bound_norm_vel(cmd['lift_up']), robot)

    if 'arm_out' in cmd:
        commands['arm'].command_stick_to_motion(bound_norm_vel(cmd['arm_out']), robot)

    if 'wrist_roll_counterclockwise' in cmd:
        commands['wrist_roll'].command_stick_to_motion(bound_norm_vel(cmd['wrist_roll_counterclockwise']), robot)

    if 'wrist_pitch_up' in cmd:
        commands['wrist_pitch'].command_stick_to_motion(bound_norm_vel(cmd['wrist_pitch_up']), robot)

    if 'wrist_yaw_counterclockwise' in cmd:
        commands['wrist_yaw'].command_stick_to_motion(bound_norm_vel(cmd['wrist_yaw_counterclockwise']), robot)

    if 'head_tilt_up' in cmd:
        commands['head_tilt'].command_stick_to_motion(bound_norm_vel(cmd['head_tilt_up']), robot)

    if 'head_pan_counterclockwise' in cmd:
        commands['head_pan'].command_stick_to_motion(bound_norm_vel(cmd['head_pan_counterclockwise']), robot)

    if 'gripper_open' in cmd:
        commands['gripper'].command_stick_to_motion(bound_norm_vel(cmd['gripper_open']), robot)


def stop_all_motion(commands, robot):
    for command in commands.values():
        command.stop_motion(robot)
