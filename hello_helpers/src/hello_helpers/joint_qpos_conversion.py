#!/usr/bin/env python3

class UnsupportedToolError(Exception):
    pass

def get_Idx(tool_name):
    """
    Returns the Idx class corresponding to the tool name.
    """
    if tool_name in ['eoa_wrist_dw3_tool_sg3', 'eoa_wrist_dw3_tool_sg3_pro']:
        return SE3_dw3_sg3_Idx
    elif tool_name == 'eoa_wrist_dw3_tool_nil':
        return eoa_wrist_dw3_tool_nil_Idx
    elif tool_name == 'eoa_wrist_dw3_tool_tablet_12in':
        return eoa_wrist_dw3_tool_tablet_12in_Idx
    elif tool_name == 'tool_stretch_dex_wrist':
        return tool_stretch_dex_wrist_Idx
    elif tool_name == 'tool_stretch_gripper':
        return tool_stretch_gripper_Idx
    elif tool_name == 'tool_none':
        return tool_none_Idx
    else:
        raise UnsupportedToolError('Undefined tool name in QposConversion.')

class SE3_dw3_sg3_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 7
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    HEAD_PAN = 5
    HEAD_TILT = 6
    BASE_TRANSLATE = 8
    BASE_ROTATE = 9

    num_joints = 10

class eoa_wrist_dw3_tool_nil_Idx:
    LIFT = 1
    ARM = 0
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    HEAD_PAN = 5
    HEAD_TILT = 6
    BASE_TRANSLATE = 7
    BASE_ROTATE = 8

    num_joints = 9

class eoa_wrist_dw3_tool_tablet_12in_Idx:
    LIFT = 1
    ARM = 0
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    HEAD_PAN = 5
    HEAD_TILT = 6
    BASE_TRANSLATE = 7
    BASE_ROTATE = 8

    num_joints = 9

class tool_stretch_dex_wrist_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 7
    WRIST_ROLL = 4
    WRIST_PITCH = 3
    WRIST_YAW = 2
    HEAD_PAN = 5
    HEAD_TILT = 6
    BASE_TRANSLATE = 8
    BASE_ROTATE = 9

    num_joints = 10

class tool_stretch_gripper_Idx:
    LIFT = 1
    ARM = 0
    GRIPPER = 3
    WRIST_YAW = 2
    HEAD_PAN = 4
    HEAD_TILT = 5
    BASE_TRANSLATE = 6
    BASE_ROTATE = 7

    num_joints = 8

class tool_none_Idx:
    LIFT = 1
    ARM = 0
    WRIST_YAW = 2
    HEAD_PAN = 3
    HEAD_TILT = 4
    BASE_TRANSLATE = 5
    BASE_ROTATE = 6

    num_joints = 7

class JointStateMapping:
    """
    Mapping of ROS joint names to the corresponding joint names in the robot.
    """
    ROS_ARM_JOINTS = ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
    ROS_LIFT_JOINT = "joint_lift"
    ROS_GRIPPER_FINGER = "joint_gripper_finger_left"
    ROS_HEAD_PAN = "joint_head_pan"
    ROS_HEAD_TILT = "joint_head_tilt"
    ROS_WRIST_YAW = "joint_wrist_yaw"
    ROS_WRIST_PITCH = "joint_wrist_pitch"
    ROS_WRIST_ROLL = "joint_wrist_roll"