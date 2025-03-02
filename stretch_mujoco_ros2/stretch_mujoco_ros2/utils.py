import math
pi = math.pi

def format_joint_pos(
    left_wheel_vel  = 0,
    right_wheel_vel = 0,
    lift            = 0,
    arm             = 0,
    wrist_yaw       = 0,
    wrist_pitch     = 0,
    wrist_roll      = 0,
    gripper         = 0,
    head_pan        = 0,
    head_tilt       = 0,
    base_rotate     = 0,
    base_translate  = 0,
) -> tuple[list, list]:
    """
    default input in order of mujoco joints
    
    return mujoco_qpos, ros_qpos
    """
    ros_jointstate_list = [arm, gripper, head_pan, head_tilt, lift, wrist_pitch, wrist_roll, wrist_yaw, base_rotate, base_translate]
    mujoco_jointstate_list = [left_wheel_vel, right_wheel_vel, lift, arm, wrist_yaw, wrist_pitch, wrist_roll, gripper, head_pan, head_tilt]
    
    ros_jointstate_list = [float(data) for data in ros_jointstate_list]
    mujoco_jointstate_list = [float(data) for data in mujoco_jointstate_list]
    return mujoco_jointstate_list, ros_jointstate_list