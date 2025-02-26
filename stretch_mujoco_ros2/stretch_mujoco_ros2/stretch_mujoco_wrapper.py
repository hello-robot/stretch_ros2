"""
Python sample script for interfacing with the Stretch Mujoco simulator
"""

import os
import threading
import time
from typing import Any, Dict, Optional

import click
import cv2
import mujoco
import mujoco.viewer
import numpy as np
from mujoco import MjData, MjModel

import stretch_mujoco.config as config
import stretch_mujoco.utils as utils

from stretch_mujoco import StretchMujocoSimulator as StretchSim

class StretchMujocoSimulator(StretchSim):
    """
    Subclassing of 
    Stretch Mujoco Simulator class for interfacing with the Mujoco simulator
    """

    def _pull_status(self) -> Dict[str, Any]:
        """
        Pull joints status of the robot from the simulator
        """
        self.status["time"] = self.mjdata.time
        self.status["lift"]["pos"] = self.mjdata.actuator("lift").length[0]
        self.status["lift"]["vel"] = self.mjdata.actuator("lift").velocity[0]
        self.status["lift"]["effort"] = self.mjdata.actuator("lift").force[0]    # effort

        self.status["arm"]["pos"] = self.mjdata.actuator("arm").length[0]
        self.status["arm"]["vel"] = self.mjdata.actuator("arm").velocity[0]
        self.status["arm"]["effort"] = self.mjdata.actuator("arm").force[0]      # effort

        self.status["head_pan"]["pos"] = self.mjdata.actuator("head_pan").length[0]
        self.status["head_pan"]["vel"] = self.mjdata.actuator("head_pan").velocity[0]
        self.status["head_pan"]["effort"] = self.mjdata.actuator("head_pan").force[0]      # effort

        self.status["head_tilt"]["pos"] = self.mjdata.actuator("head_tilt").length[0]
        self.status["head_tilt"]["vel"] = self.mjdata.actuator("head_tilt").velocity[0]
        self.status["head_tilt"]["effort"] = self.mjdata.actuator("head_tilt").force[0]      # effort

        self.status["wrist_yaw"]["pos"] = self.mjdata.actuator("wrist_yaw").length[0]
        self.status["wrist_yaw"]["vel"] = self.mjdata.actuator("wrist_yaw").velocity[0]
        self.status["wrist_yaw"]["effort"] = self.mjdata.actuator("wrist_yaw").force[0]      # effort

        self.status["wrist_pitch"]["pos"] = self.mjdata.actuator("wrist_pitch").length[0]
        self.status["wrist_pitch"]["vel"] = self.mjdata.actuator("wrist_pitch").velocity[0]
        self.status["wrist_pitch"]["effort"] = self.mjdata.actuator("wrist_pitch").force[0]      # effort
        

        self.status["wrist_roll"]["pos"] = self.mjdata.actuator("wrist_roll").length[0]
        self.status["wrist_roll"]["vel"] = self.mjdata.actuator("wrist_roll").velocity[0]
        self.status["wrist_roll"]["effort"] = self.mjdata.actuator("wrist_roll").force[0]      # effort

        real_gripper_pos = self._to_real_gripper_range(self.mjdata.actuator("gripper").length[0])
        self.status["gripper"]["pos"] = real_gripper_pos
        self.status["gripper"]["vel"] = self.mjdata.actuator("gripper").velocity[0]  # This is still in sim gripper range
        self.status["gripper"]["effort"] = self.mjdata.actuator("gripper").force[0]      # effort
        
        # end_of_arm collection
        self.status['end_of_arm'] = dict(
            wrist_yaw = self.status["wrist_yaw"],
            wrist_pitch = self.status["wrist_pitch"],
            wrist_roll = self.status["wrist_roll"],
            stretch_gripper = self.status["gripper"],
        )
        # head collection
        self.status['head'] = dict(
            head_pan = self.status["head_pan"],
            head_tilt = self.status["head_tilt"],
        )
        
        left_wheel_vel = self.mjdata.actuator("left_wheel_vel").velocity[0]
        right_wheel_vel = self.mjdata.actuator("right_wheel_vel").velocity[0]

        (
            self.status["base"]["x"],
            self.status["base"]["y"],
            self.status["base"]["theta"],
        ) = self.get_base_pose()
        (
            self.status["base"]["x_vel"],
            self.status["base"]["theta_vel"],
        ) = self.diff_drive_fwd_kinematics(left_wheel_vel, right_wheel_vel)
        self.status["base"]["y_vel"] = 0.0
        
        return self.status
    
    # def get_actuator_names(self):
    #     """
    #     Get the names of all actuators in the model.
        
    #     Returns:
    #         list: The names of all actuators in the model. list[id] = actuator_name
    #     """
    #     num_actuators = self.mjmodel.nu  # Number of actuators
    #     actuator_names = []

    #     for i in range(num_actuators):
    #         name = mujoco.mj_id2name(self.mjmodel, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    #         actuator_names.append(name)
    #         # print(f"Actuator {i}: {name}")
    #     return actuator_names

    def update_urdf(self):
        raise NotImplementedError
        self.urdf_model = utils.URDFmodel()


@click.command()
@click.option(
    "--scene-xml-path", default=utils.default_scene_xml_path, help="Path to the scene xml file"
)
@click.option("--headless", is_flag=True, help="Run the simulation headless")
def main(
    scene_xml_path: str,
    headless: bool,
) -> None:
    robot_sim = StretchMujocoSimulator(scene_xml_path)
    robot_sim.start(headless=headless)
    # display camera feeds
    try:
        while robot_sim.is_running():
            camera_data = robot_sim.pull_camera_data()
            cv2.imshow("cam_d405_rgb", cv2.cvtColor(camera_data["cam_d405_rgb"], cv2.COLOR_RGB2BGR))
            cv2.imshow("cam_d405_depth", camera_data["cam_d405_depth"])
            cv2.imshow(
                "cam_d435i_rgb", cv2.cvtColor(camera_data["cam_d435i_rgb"], cv2.COLOR_RGB2BGR)
            )
            cv2.imshow("cam_d435i_depth", camera_data["cam_d435i_depth"])
            cv2.imshow("cam_nav_rgb", cv2.cvtColor(camera_data["cam_nav_rgb"], cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                break
    except KeyboardInterrupt:
        robot_sim.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    import warnings
    warnings.warn("use 'python -m stretch_mujoco', not 'python -m stretch_mujoco.stretch_mujoco'", DeprecationWarning)

    # Check if we are on macOS
    if os.uname().sysname == "Darwin":
        print("macOS detected. Please use the following command to run the simulator:")
        print("python3 -m stretch_mujoco")
    else:
        main()
