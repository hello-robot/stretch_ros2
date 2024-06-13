#! /usr/bin/env python3
import stretch_body.robot
from stretch_body.hello_utils import *
import time
import numpy as np
import stretch_body.stretch_gripper
# from stretch_wbc_pkg.scope import Oscilloscope

import ikpy.urdf.utils
from IPython import display
import ipywidgets as widgets
from ikpy.chain import Chain
import urdfpy
import numpy as np
from stretch_wbc_pkg.ik_utils import ik_utils

class StretchArmIK:

    def __init__(self, robot, cfg, targets):
        self.robot = robot
        # self.urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        self.urdf_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/modified_urdf/stretch_modified.urdf'
        self.cfg = cfg
        self.vis_tree = cfg['vis_tree']
        self.check_urdf = cfg['check_urdf']
        self.modify_urdf = cfg['modify_urdf']
        self.ik_utils = ik_utils(self.urdf_path)
        self.target_position = targets['target_point']
        self.target_orientation = targets['target_orientation']
        self.pretarget_orientation = targets['pretarget_orientation']
        
    def execute(self, robot):
        print('Executing')
        if not robot.startup():
            print('Failed to startup')
        
        if not robot.is_homed():
            robot.home()

        if self.modify_urdf:
            self.ik_utils.modify_urdf()

        if self.check_urdf: #sanity check
            self.ik_utils.check_urdf()

        chain = Chain.from_urdf_file(self.urdf_path)

        for link, active in zip(chain.links, chain.active_links_mask):
            if active:
                print(link.name)

        q_init = self.ik_utils.get_current_configuration(robot)
        # print(f'Initial joint angles: {q_init}')
        self.ik_utils.print_joint_angles(q_init)

        chain = ikpy.chain.Chain.from_urdf_file(self.urdf_path)
        # # # q_soln = chain.inverse_kinematics(self.target_position, q_init)

        q_mid = chain.inverse_kinematics(self.target_position, self.pretarget_orientation, orientation_mode='all', initial_position=q_init, optimizer='scalar', max_iter=1000)
        q_soln = chain.inverse_kinematics(self.target_position, self.target_orientation, orientation_mode='all', initial_position=q_mid,  optimizer='scalar', max_iter=1000)

        # self.ik_utils.print_joint_angles(q_soln)

        if self.vis_tree:
            self.ik_utils.visualize_chain(q_soln, self.target_position, self.target_orientation)

        error = self.ik_utils.ik_error(q_soln, self.target_position)
        print(f'Error: {error}')


def main():
    robot = stretch_body.robot.Robot()
    cfg = {
        'vis_tree': True,  # Or False, depending on what you want
        'check_urdf': True,  
        'modify_urdf': False,
        # Other cfg parameters...
    }
    targets = {
        'target_point': [0.2, -1.441, 0.654],
        'target_orientation': ikpy.utils.geometry.rpy_matrix(0.0, 0.0, np.deg2rad(-90)),
        'pretarget_orientation': ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0)
    }

    arm = StretchArmIK(robot, cfg, targets)
    arm.execute(robot)

    print('done')

if __name__ == '__main__':
    main()




