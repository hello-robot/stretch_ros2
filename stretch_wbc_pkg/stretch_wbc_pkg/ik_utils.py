import urdfpy
import ikpy
from ikpy.chain import Chain
from IPython import display
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import ikpy.utils.plot as plot_utils
import pathlib
import stretch_body.hello_utils as hu
from scipy.spatial.transform import Rotation as R


class ik_utils:
    def __init__(self, urdf_path):
        # Pamaeters
        self.urdf_path = urdf_path
        self.original_urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        self.modified_urdf_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/modified_urdf/stretch_modified.urdf'
        self.chain = Chain.from_urdf_file(self.urdf_path)
        # self.save_plot_path = '/home/hello-robot/ament_ws/stretch_wbc_plots'

    def check_urdf(self,):
        urdf = urdfpy.URDF.load(self.urdf_path)
        links = [link.name for link in urdf.links]
        links_to_check = ['link_wrist_yaw', 'link_wrist_pitch', 'link_wrist_roll']
        
        # Print information about presence of specific links
        for link in links_to_check:
            if link in links:
                print(f"{link} is present in the URDF.")
            else:
                print(f"{link} is not present in the URDF.")

        for joint in urdf.joints:
            if joint.name == 'joint_base_translation':
                print(f"{joint.name} is a {joint.joint_type} joint.")
            if joint.name == 'joint_base_rotation':
                print(f"{joint.name} is a {joint.joint_type} joint.")


    def print_tree(self,):
        # Display the robot tree
        print("Displaying the robot tree for")
        tree = ikpy.urdf.utils.get_urdf_tree(self.urdf_path, "base_link")[0]
        display.display_png(tree)

    def print_joint_angles(self, joint_angles):
        for link, active, angle in zip(self.chain.links, self.chain.active_links_mask, joint_angles):
            if active:
                print(f'Joint {link.name}: {angle}')

    def visualize_chain(self, joint_angles, target_point=None, target_orientation=None):
        print("Visualizing the chain...")
        
        fig, ax = plot_utils.init_3d_figure()
        plt.xlim(-0.2, 0.2)
        plt.ylim(-0.75, 0.2)
        ax.set_zlim(0.0, 1.0)
        self.chain.plot(joint_angles, ax, target=target_point)
        inital_base_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, 0.0) # can only roatte about z axis
        target_base_orientation = ikpy.utils.geometry.rpy_matrix(0.0, 0.0, joint_angles[1]) 
        origin_point = np.array([0.0, 0.0, 0.0])

        for i, color in enumerate(['r', 'g', 'b']):
            ax.quiver(*target_point, *target_orientation[:, i], color=color, length=0.1)
        
        # Plot initial_base_orientation
        for i, color in enumerate(['r', 'g', 'b']):
            ax.quiver(*origin_point, *inital_base_orientation[:, i], color=color, length=0.1, alpha=0.5)

        # Plot target_base_orientation
        for i, color in enumerate(['r', 'g', 'b']):
            ax.quiver(*origin_point, *target_base_orientation[:, i], color=color, length=0.1)

        plt.show()

    def bound_range(self,name, value):
        names = [l.name for l in self.chain.links]
        index = names.index(name)
        bounds = self.chain.links[index].bounds
        return min(max(value, bounds[0]), bounds[1])

    def get_current_configuration(self, robot):
        q_base_trans = 0.0
        q_base_rot = 0.0
        q_lift = self.bound_range('joint_lift', robot.lift.status['pos'])
        q_arml = self.bound_range('joint_arm_l0', robot.arm.status['pos'] / 4.0)
        q_yaw = self.bound_range('joint_wrist_yaw', robot.end_of_arm.status['wrist_yaw']['pos'])
        q_pitch = self.bound_range('joint_wrist_pitch', robot.end_of_arm.status['wrist_pitch']['pos'])
        q_roll = self.bound_range('joint_wrist_roll', robot.end_of_arm.status['wrist_roll']['pos'])
        return [0.0, q_base_rot, q_base_trans, 0.0, q_lift, 0.0, q_arml, q_arml, q_arml, q_arml, q_yaw, 0.0, q_pitch, q_roll, 0.0, 0.0]
    
    def ik_error(self, ik_sol, target_point):
        # calculate the error
        error = np.linalg.norm(self.chain.forward_kinematics(ik_sol)[:3, 3] - target_point)
        return error

    def add_base_virtual_joints(self, modified_urdf):
        # Define the joint_base_translation joint
        joint_base_translation = urdfpy.Joint(
            name='joint_base_translation',
            parent='link_base_rotation',
            child='link_base_translation',
            joint_type='prismatic',
            axis=np.array([1.0, 0.0, 0.0]),
            origin=np.eye(4, dtype=np.float64),
            limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.0, upper=1.0)
        )

        # Define the joint_base_rotation joint
        joint_base_rotation = urdfpy.Joint(
            name='joint_base_rotation',
            parent='base_link',
            child='link_base_rotation',
            joint_type='revolute',
            axis=np.array([0.0, 0.0, 1.0]),  # Rotating around the Z-axis
            origin=np.eye(4, dtype=np.float64),
            limit=urdfpy.JointLimit(effort=100.0, velocity=1.0, lower=-1.5708, upper=1.5708)
        )
        
        modified_urdf._joints.append(joint_base_translation)
        modified_urdf._joints.append(joint_base_rotation)

        link_base_translation = urdfpy.Link(name='link_base_translation',
                                            inertial=None,
                                            visuals=None,
                                            collisions=None)
        
        link_base_rotation = urdfpy.Link(name='link_base_rotation',
                                            inertial=None,
                                            visuals=None,
                                            collisions=None)
        
        modified_urdf._links.append(link_base_translation)
        modified_urdf._links.append(link_base_rotation)

        for j in modified_urdf._joints:
            if j.name == 'joint_mast':
                j.parent = 'link_base_translation'

        print(f"name: {modified_urdf.name}")
        print(f"num links: {len(modified_urdf.links)}")
        print(f"num joints: {len(modified_urdf.joints)}")
        print("added fake links and joints and saved the modified URDF to {self.modified_urdf_path}")

        modified_urdf.save(self.modified_urdf_path)

    def modify_urdf(self,):
        original_urdf = urdfpy.URDF.load(self.original_urdf_path)
        print(f"name: {original_urdf.name}")
        print(f"num links: {len(original_urdf.links)}")
        print(f"num joints: {len(original_urdf.joints)}")

        original_links = [link.name for link in original_urdf.links]

        modified_urdf = original_urdf.copy()
        names_of_links_to_remove = ['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'laser', 'respeaker_base']
        links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
        for lr in links_to_remove:
            modified_urdf._links.remove(lr)
        names_of_joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'joint_laser', 'joint_respeaker']
        joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
        for jr in joints_to_remove:
            modified_urdf._joints.remove(jr)
        
        print("removed unnecessary links and joints...")
        print(f"name: {modified_urdf.name}")
        print(f"num links: {len(modified_urdf.links)}")
        print(f"num joints: {len(modified_urdf.joints)}")

        self.add_base_virtual_joints(modified_urdf)