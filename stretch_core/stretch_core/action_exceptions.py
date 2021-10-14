from control_msgs.action import FollowJointTrajectory


# Exceptions for dealing with FollowJointTrajectory
class FollowJointTrajectoryException(RuntimeError):
    """Parent class for all FollowJointTrajectory errors.

    Each subclass should define the CODE based on the constants in FollowJointTrajectory.Result.
    """

    CODE = -100  # Arbitrary constant for unknown error.


class InvalidGoalException(FollowJointTrajectoryException):
    CODE = FollowJointTrajectory.Result.INVALID_GOAL


class InvalidJointException(FollowJointTrajectoryException):
    CODE = FollowJointTrajectory.Result.INVALID_JOINTS

