#!/usr/bin/env python3

import math


class GripperConversion:
    """Convert between Stretch gripper actuator, aperture, and finger units.

    The default geometry matches ``stretch_gripper.xacro``: the fingertip is
    90% of 0.19011 m from the finger joint.  Parameters remain injectable so a
    robot description with different geometry can use the same conversion
    without changing this helper.

    Conversion methods intentionally do not clamp values.  Hardware limits
    depend on the calibrated robot and are enforced by the command layer.
    """

    DEFAULT_FINGER_LENGTH_M = 0.9 * 0.19011
    DEFAULT_OPEN_APERTURE_M = 0.09
    DEFAULT_CLOSED_APERTURE_M = 0.0
    DEFAULT_OPEN_ROBOTIS = 70.0
    DEFAULT_CLOSED_ROBOTIS = 0.0

    def __init__(
        self,
        finger_length_m=DEFAULT_FINGER_LENGTH_M,
        open_aperture_m=DEFAULT_OPEN_APERTURE_M,
        closed_aperture_m=DEFAULT_CLOSED_APERTURE_M,
        open_robotis=DEFAULT_OPEN_ROBOTIS,
        closed_robotis=DEFAULT_CLOSED_ROBOTIS,
    ):
        parameters = {
            "finger_length_m": finger_length_m,
            "open_aperture_m": open_aperture_m,
            "closed_aperture_m": closed_aperture_m,
            "open_robotis": open_robotis,
            "closed_robotis": closed_robotis,
        }
        for name, value in parameters.items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")

        if finger_length_m <= 0.0:
            raise ValueError("finger_length_m must be positive")
        if open_aperture_m <= closed_aperture_m:
            raise ValueError("open_aperture_m must be greater than closed_aperture_m")
        if open_robotis <= closed_robotis:
            raise ValueError("open_robotis must be greater than closed_robotis")

        self.finger_length_m = finger_length_m
        self.open_aperture_m = open_aperture_m
        self.closed_aperture_m = closed_aperture_m
        self.open_robotis = open_robotis
        self.closed_robotis = closed_robotis

        self.robotis_to_aperture_slope = (
            (self.open_aperture_m - self.closed_aperture_m)
            / (self.open_robotis - self.closed_robotis)
        )

    def robotis_to_aperture(self, robotis_in):
        """Convert actuator percentage to aperture in metres."""
        return (
            self.robotis_to_aperture_slope * (robotis_in - self.closed_robotis)
        ) + self.closed_aperture_m

    def aperture_to_robotis(self, aperture_m):
        """Convert aperture in metres to actuator percentage."""
        return (
            (aperture_m - self.closed_aperture_m)
            / self.robotis_to_aperture_slope
        ) + self.closed_robotis

    def aperture_to_finger_rad(self, aperture_m):
        """Convert aperture in metres to the angle of either finger."""
        return aperture_m / (2.0 * self.finger_length_m)

    def finger_rad_to_aperture(self, finger_rad):
        """Convert the angle of either finger to aperture in metres."""
        return 2.0 * finger_rad * self.finger_length_m

    def finger_to_robotis(self, finger_ang_rad):
        aperture_m = self.finger_rad_to_aperture(finger_ang_rad)
        return self.aperture_to_robotis(aperture_m)

    def robotis_to_finger(self, robotis_pct):
        aperture_m = self.robotis_to_aperture(robotis_pct)
        return self.aperture_to_finger_rad(aperture_m)

    def status_to_all(self, gripper_status):
        aperture_m = self.robotis_to_aperture(gripper_status["pos_pct"])
        finger_rad = self.aperture_to_finger_rad(aperture_m)
        finger_effort = gripper_status["effort"]
        # Preserve the existing status conversion.  ``vel`` is supplied by
        # Stretch Body in actuator rad/s, so a physically exact fingertip
        # velocity also needs the robot-specific actuator-to-pct derivative.
        finger_vel = (
            self.robotis_to_aperture_slope * gripper_status["vel"]
        ) / 2.0
        return aperture_m, finger_rad, finger_effort, finger_vel
