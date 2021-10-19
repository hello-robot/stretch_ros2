from hello_helpers.gripper_conversion import GripperConversion
from hello_helpers.hello_misc import to_sec, to_transform, transform_to_triple, twist_to_pair


class TrajectoryComponent:
    def __init__(self, name, trajectory_manager):
        self.name = name
        self.trajectory_manager = trajectory_manager

    def pretty_print(self):
        return repr(self.trajectory_manager.trajectory)

    def get_position(self):
        return self.trajectory_manager.status['pos']

    def get_velocity(self):
        return self.trajectory_manager.status['vel']

    def get_desired_position_at(self, dt):
        pos, _, _ = self.trajectory_manager.trajectory.evaluate_at(dt)
        return pos

    def add_waypoints(self, waypoints, index):
        self.trajectory_manager.trajectory.clear()
        for waypoint in waypoints:
            t = to_sec(waypoint.time_from_start)
            x = waypoint.positions[index]
            v = waypoint.velocities[index] if index < len(waypoint.velocities) else None
            a = waypoint.accelerations[index] if index < len(waypoint.accelerations) else None
            self.add_waypoint(t, x, v, a)
        if len(self.trajectory_manager.trajectory) > 0:
            self.trajectory_manager.trajectory.pop()
            last_waypoint = waypoints[-1]
            t = to_sec(last_waypoint.time_from_start)
            x = last_waypoint.positions[index]
            v = last_waypoint.velocities[index] if index < len(last_waypoint.velocities) else None
            a = last_waypoint.accelerations[index] if index < len(last_waypoint.accelerations) else None
            self.add_waypoint(t, x, v, a)

    def add_waypoint(self, t, x, v, a):
        self.trajectory_manager.trajectory.add(t, x, v, a)


class HeadPanComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_head_pan', robot.head.get_joint('head_pan'))


class HeadTiltComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_head_tilt', robot.head.get_joint('head_tilt'))


class WristYawComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_wrist_yaw', robot.end_of_arm.get_joint('wrist_yaw'))


class GripperComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'stretch_gripper', robot.end_of_arm.get_joint('stretch_gripper'))
        self.gripper_conversion = GripperConversion()

        # Convenient aliases
        self.finger_rad_to_robotis = self.gripper_conversion.finger_to_robotis
        self.robotis_to_finger_rad = self.gripper_conversion.robotis_to_finger

    def get_position(self):
        return self.robotis_to_finger_rad(self.trajectory_manager.status['pos_pct'])

    def get_desired_position_at(self, dt):
        pos, _, _ = self.trajectory_manager.trajectory.evaluate_at(dt)
        return self.robotis_to_finger_rad(pos)

    def add_waypoint(self, t, x, v, a):
        x = self.finger_rad_to_robotis(x)
        # v = self.finger_rad_to_robotis(v) if v is not None else None
        # a = self.finger_rad_to_robotis(a) if a is not None else None
        self.trajectory_manager.trajectory.add(t, x, None, None)


class ArmComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'wrist_extension', robot.arm)

    def add_waypoint(self, t, x, v, a):
        # v = None
        a = None
        self.trajectory_manager.trajectory.add(t, x, v, a)


class LiftComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_lift', robot.lift)

    def add_waypoint(self, t, x, v, a):
        # v = None
        a = None
        self.trajectory_manager.trajectory.add(t, x, v, a)


class BaseComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'position', robot.base)

    def get_position(self):
        return to_transform(self.trajectory_manager.status)

    def get_velocity(self):
        raise NotImplementedError()

    def get_desired_position_at(self, dt):
        # TODO: Fill in actual interpolated position
        return to_transform({'x': 0.0, 'y': 0.0, 'theta': 0.0})

    def add_waypoints(self, waypoints, index):
        self.trajectory_manager.trajectory.clear()
        for waypoint in waypoints:
            t = to_sec(waypoint.time_from_start)
            x = waypoint.transforms[index]
            v = waypoint.velocities[index] if index < len(waypoint.velocities) else None
            a = waypoint.accelerations[index] if index < len(waypoint.accelerations) else None
            self.add_waypoint(t, x, v, a)
        if len(self.trajectory_manager.trajectory) > 0:
            self.trajectory_manager.trajectory.pop()
            last_waypoint = waypoints[-1]
            t = to_sec(last_waypoint.time_from_start)
            x = last_waypoint.transforms[index]
            v = last_waypoint.velocities[index] if index < len(last_waypoint.velocities) else None
            a = last_waypoint.accelerations[index] if index < len(last_waypoint.accelerations) else None
            self.add_waypoint(t, x, v, a)

    def add_waypoint(self, t, x, v, a):
        x = transform_to_triple(x)
        v = twist_to_pair(v) if v is not None else (None, None)
        a = twist_to_pair(a) if a is not None else (None, None)
        self.trajectory_manager.trajectory.add(t, x[0], x[1], x[2], v[0], v[1], a[0], a[1])


def get_trajectory_components(robot):
    return {component.name: component for component in [HeadPanComponent(robot),
                                                        HeadTiltComponent(robot),
                                                        WristYawComponent(robot),
                                                        GripperComponent(robot),
                                                        ArmComponent(robot),
                                                        LiftComponent(robot),
                                                        BaseComponent(robot),
                                                        ]}