from hello_helpers.hello_misc import to_sec


class TrajectoryComponent:
    def __init__(self, name, trajectory_manager):
        self.name = name
        self.trajectory_manager = trajectory_manager

    def get_position(self):
        return self.trajectory_manager.status['pos']

    def get_velocity(self):
        return self.trajectory_manager.status['vel']

    def get_desired_position_at(self, dt):
        return self.trajectory_manager.trajectory.evaluate_at(dt).position

    def add_waypoints(self, waypoints, index):
        self.trajectory_manager.trajectory.clear_waypoints()
        for waypoint in waypoints:
            t = to_sec(waypoint.time_from_start)
            x = waypoint.positions[index]
            v = waypoint.velocities[index] if index < len(waypoint.velocities) else None
            a = waypoint.accelerations[index] if index < len(waypoint.accelerations) else None
            self.add_waypoint(t, x, v, a)

    def add_waypoint(self, t, x, v, a):
        self.trajectory_manager.trajectory.add_waypoint(t, x, v, a)


class HeadPanComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_head_pan', robot.head.get_joint('head_pan'))


class HeadTiltComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_head_tilt', robot.head.get_joint('head_tilt'))


class WristYawComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_wrist_yaw', robot.end_of_arm.motors['wrist_yaw'])


class ArmComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'wrist_extension', robot.arm)


class LiftComponent(TrajectoryComponent):
    def __init__(self, robot):
        TrajectoryComponent.__init__(self, 'joint_lift', robot.lift)


def get_trajectory_components(robot):
    return {component.name: component for component in [HeadPanComponent(robot),
                                                        HeadTiltComponent(robot),
                                                        WristYawComponent(robot),
                                                        ArmComponent(robot),
                                                        LiftComponent(robot),
                                                        ]}
