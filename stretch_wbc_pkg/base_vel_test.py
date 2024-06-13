#! /usr/bin/env python3
import stretch_body.robot
from stretch_body.hello_utils import *
import time
import numpy as np
import stretch_body.stretch_gripper
        

from stretch_wbc_pkg.scope import Oscilloscope

class BaseVel:

    def __init__(self, robot, interval = 1, total_time = 20.0, run = False, vis = None):

        self.robot = robot
        self.arm_init = 0.0
        self.lift_init = 0.0
        self.total_time = total_time
        self.interval = interval * 3
        self.radius = 1.0 # meters
        self.base_waypoints = []
        self.fullbody_waypoints = []
        self.arm_waypoints = np.zeros((self.interval, 2))
        self.lift_waypoints = np.zeros((self.interval, 2))
        self.head_waypoints = np.zeros((self.interval, 3))
        self.gripper_waypoints = np.zeros((self.interval, 2))
        self.Jointlimits = {
            "arm_m": [0.0, 0.25],
            "lift_m": [0.2, 0.7],
            "head_tilt": [-1.57, 1.57],
            "head_pan": [-3.14, 3.14],
            "wrist": [90.0, -45.0], # End of arm pos (rad)
            "gripper_pitch": [0.1, 0.5], # gripper pos (rad)
            "gripper_roll": [0.1, 0.5], # gripper pos (rad)
            
        }
        self.vis = vis
        self.oscilloscope = Oscilloscope(self.Jointlimits, self.interval, self.total_time)

        self.gripper = stretch_body.stretch_gripper.StretchGripper()

        self.run = run

    def compute_pos_and_timesteps(self, min_pos, max_pos, total_time, num_steps):
        # Ensure num_steps is a multiple of 3
        if num_steps % 3 != 0:
            raise ValueError("num_steps must be a multiple of 3")
        
        # Generate timesteps
        timesteps = np.linspace(0, total_time, num_steps)
        
        # Generate positions following the pattern [min, max, min]
        pattern = [min_pos, max_pos, min_pos]
        pos = np.tile(pattern, num_steps // 3)
        
        return pos, timesteps
        
    
    def base_init_pose(self):
        return 0.0 , 0.0, 0.0
        # return self.robot.base.status['x'], self.robot.base.status['y'], self.robot.base.status['theta']
    
    
    def compute_arm_waypoints(self, total_time, interval):
        joint_name = 'arm'
        num_steps = interval
        min_pos, max_pos = self.Jointlimits['arm_m']
        timesteps = np.linspace(0, total_time, num_steps)

        pos, timesteps = self.compute_pos_and_timesteps(min_pos, max_pos, total_time, num_steps)

        waypoints = []
        for i in range(num_steps):
            position = pos[i]
            velocity = 0.0
            waypoints.append((timesteps[i], position, velocity))

        if self.vis is not None:
            self.oscilloscope.plot_input_vs_time(waypoints, joint_name, self.vis)
        
        return waypoints
    
        
    def compute_base_twist2d_waypoints(self, total_time, interval):
        joint_name = 'arm'
        num_steps = interval
        min_l_vel, max_l_vel = self.Jointlimits['base_linear_vel']
        min_a_vel, max_a_vel = self.Jointlimits['base_ang_vel']
        timesteps = np.linspace(0, total_time, num_steps)

        l_vel, timesteps = self.compute_pos_and_timesteps(min_l_vel, max_l_vel, total_time, num_steps)
        a_vel, timesteps = self.compute_pos_and_timesteps(min_l_vel, max_l_vel, total_time, num_steps)

        waypoints = []
        for i in range(num_steps):
            l_velocity = l_vel[i]
            a_velocity = a_vel[i]
            waypoints.append((timesteps[i], position, velocity))

        if self.vis is not None:
            self.oscilloscope.plot_input_vs_time(waypoints, joint_name, self.vis)
        
        return waypoints
    



    def compute_base_waypoints(self, total_time, interval, distance):
        x0, y0, theta0 = self.base_init_pose()
        print(f'Initial Pose: {x0}, {y0}, {theta0}')
        
        timesteps = np.linspace(0, total_time, interval)

        waypoints = []

        # Compute waypoints
        for t in timesteps:
            velocity = (0.0, 0.0)
            if t <= total_time / 2:
                # Moving forward
                progress = (t / (total_time / 2)) * distance
                
                waypoints.append((t, (x0 + progress * np.cos(theta0), y0 + progress * np.sin(theta0), theta0), velocity))
            else:
                # Returning back
                progress = ((total_time - t) / (total_time / 2)) * distance
                waypoints.append((t, (x0 + progress * np.cos(theta0), y0 + progress * np.sin(theta0), theta0), velocity))

        return waypoints


        
    def compute_head_waypoints(self, total_time, interval):
        num_steps = interval
        timesteps = np.linspace(0, total_time, num_steps)
        inital_tilt, initial_pan = self.head_init_pose()
        waypoints = []
        for i in range(num_steps):
            position = (inital_tilt, initial_pan)
            velocity = 0.0
            waypoints.append((timesteps[i], position, velocity))
        return waypoints
    
    
        

    def send_base_waypoints(self, waypoint):
        t, pos, vel = waypoint
        # print(f'Base position at time {t}: {pos}')
        self.robot.base.trajectory.add(time=t, x=pos[0], y=pos[1], theta=deg_to_rad(0.0), translational_vel=vel[0], rotational_vel=vel[1])

    def execute(self,):
        
        num_steps = self.interval
        print('Time: %f'%(self.total_time))
        print('Number of steps: %d'%(num_steps))
        distance = 0.5
        # print('move all joints to initial positions')
        # self.robot.arm.move_to(0.0)
        # self.robot.lift.move_to(0.2)
        # self.robot.push_command()
        # self.robot.head.pose('ahead')
        # self.robot.end_of_arm.motors['wrist_yaw'].pose('side')
        # time.sleep(4.0)
        
        self.base_waypoints = self.compute_base_waypoints(self.total_time, num_steps, distance)


        self.oscilloscope.print_base_waypoints(self.base_waypoints)
        # self.oscilloscope.plot_base_waypoints(self.base_waypoints)

        if self.run is True:

            for i in range(num_steps):
                print('Sending waypoints')
                # print('arm position at time %f: %f' % (self.arm_waypoints[i][0], self.arm_waypoints[i][1]))
                
                self.send_base_waypoints(self.base_waypoints[i])
                self.send_arm_waypoints(self.arm_waypoints[i]) # issue with the arm (does not always hit the waypoints)
                self.send_lift_waypoints(self.lift_waypoints[i])
                self.send_head_waypoints(self.head_waypoints[i])
                self.send_wrist_waypoints(self.wrist_waypoints[i])
            # self.robot.base.trajectory.add(time=0, x=0.0, y=0.0, theta=deg_to_rad(0.0), translational_vel=0.0, rotational_vel=0.0)  
            # self.robot.base.trajectory.add(time=10, x=0.0, y=0.0, theta=deg_to_rad(45.0), translational_vel=0.0, rotational_vel=0.0)    
            # self.robot.base.trajectory.add(time=20, x=0.0, y=0.0, theta=deg_to_rad(0.0), translational_vel=0.0, rotational_vel=0.0)    
            # self.send_gripper_waypoints(self.gripper_waypoints, self.robot)

            self.robot.follow_trajectory()
            # ts = time.time()
            while self.robot.is_trajectory_active():
                #dt = (time.time() - ts) / 20.0  # 0-1
                # print('Time remaining: %f'%(20.0-(time.time()-ts)))
                # r.base.set_translate_velocity(v_m=dt*0.5) #Uncomment this for base motion (put up on blocks first!)
                #r.push_command()
                time.sleep(0.1)

            self.robot.stop_trajectory()
            print('Done')

        else :
            # self.oscilloscope.plot_input_vs_time(self.lift_waypoints, joint_name, vis='plot')
            print('Not running')
            


def main():
    robot = stretch_body.robot.Robot()
    robot.startup()
    run = True
    interval = 1
    time = 20.0
    vis = None
    # vis = 'plot'
    base = BaseVel(robot, interval, time, run, vis)
    base.execute()
    robot.stop()
    print('done')

if __name__ == '__main__':
    main()




