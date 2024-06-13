import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

class Oscilloscope:
    def __init__(self, BaseJointlimits, interval, total_time):
        # Pamaeters
        # Joint Limits
        # self.x_limits = BaseJointlimits["base_x"] # position x
        # self.y_limits = BaseJointlimits["base_y"] # position y
        # self.theta_limits = BaseJointlimits["base_theta"] # angle
        # self.trans_vel_limits = BaseJointlimits["base_trans_vel"] # translation velocity
        # self.rot_vel_limits = BaseJointlimits["base_rot_vel"]
        self.global_interval = interval
        self.total_time = total_time
        self.save_plot_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots'

    def plot_input_vs_time(self, waypoints, joint_name, vis=None):
        if joint_name != 'base':
            # Plotting
            # Plotting
            pos_np = np.array([waypoint[1] for waypoint in waypoints])
            vel_np = np.array([waypoint[2] for waypoint in waypoints])
            timesteps_np = np.array([waypoint[0] for waypoint in waypoints])
            print(f'Time steps: {timesteps_np}')
            print(f'Position: {pos_np}')
            cs = CubicSpline(timesteps_np, pos_np)

            # Generate smooth points for plotting
            smooth_timesteps = np.linspace(timesteps_np.min(), timesteps_np.max(), 500)
            smooth_pos = cs(smooth_timesteps)

            plt.plot(smooth_timesteps, smooth_pos, 'b-', label='Interpolated')
            plt.plot(timesteps_np, pos_np, 'ro')  # Red dots for original points
            plt.xlabel('Timesteps')
            plt.ylabel('Position')
            plt.title(f'{joint_name} Position vs Timesteps')
            if vis == 'plot':
                plt.show()
            elif vis == 'save':
                # plt.savefig(f'{joint_name}_position_vs_timesteps.png')
                plt.savefig(f'{self.save_plot_path}/{joint_name}_plot.png')
                print(f'{joint_name} plot saved at {self.save_plot_path}')
            else:
                print('Invalid visualization option')
        else:
            print('Plotting Base Joint')

    def print_base_waypoints(self, waypoints):
        print(f"{'Time':>8} {'X':>8} {'Y':>8} {'Theta':>8} {'Trans. Vel.':>12} {'Rot. Vel.':>12}")
        print("="*56)
        for t, pos, vel in waypoints:
            x, y, theta = pos
            trans_vel, rot_vel = vel
            print(f"{t:8.2f} {x:8.2f} {y:8.2f} {theta:8.2f} {trans_vel:12.2f} {rot_vel:12.2f}")

    def plot_base_waypoints(self, waypoints):
        fig, ax = plt.subplots()
        ax.set_title('Base Waypoints')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')

        x_points = [pos[0] for _, pos, _ in waypoints]
        y_points = [pos[1] for _, pos, _ in waypoints]
        theta_points = [pos[2] for _, pos, _ in waypoints]

        ax.plot(x_points, y_points, 'ro')

        for i, (t, pos, _) in enumerate(waypoints):
            x, y, theta = pos
            ax.annotate(f'{t:.2f}', (x, y), textcoords="offset points", xytext=(5, 5), ha='center')
            dx = np.cos(theta) * 0.5  # Scale arrow length for better visibility
            dy = np.sin(theta) * 0.5
            ax.arrow(x, y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')

        plt.grid(True)
        plt.show()

    #     self.interval = interval
    #     # self.sense_frequency = sense_frequency
    #     self.total_time = total_time

    #     # Initial data
    #     self.x_data = np.linspace(self.x_min, self.x_max, 100)
    #     self.y_data = np.zeros_like(self.x_data)

    #     # Setup plot and widgets
    #     self.fig, self.ax = plt.subplots()
    #     plt.subplots_adjust(left=0.1, bottom=0.25)
    #     self.l, = plt.plot(self.x_data, self.y_data, lw=2)
    #     self.ax.set_ylim(0, self.y_max)
    #     self.ax.set_xlim(self.x_min, self.x_max)
    #     self.ax.set_xlabel('Input')
    #     self.ax.set_ylabel('Time (s)')
    #     self.ax.grid(True)

    #     # Add sliders
    #     self.ax_slider_x = plt.axes([0.1, 0.1, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    #     self.slider_x = Slider(self.ax_slider_x, 'X Value', self.x_min, self.x_max, valinit=self.x_min)

    #     self.ax_slider_y = plt.axes([0.1, 0.05, 0.8, 0.03], facecolor='lightgoldenrodyellow')
    #     self.slider_y = Slider(self.ax_slider_y, 'Time Interval', 0.1, self.y_max, valinit=self.interval)

    #     # Add button
    #     self.ax_button = plt.axes([0.8, 0.01, 0.1, 0.04])
    #     self.button = Button(self.ax_button, 'Reset')

    #     # Connect the sliders and button to their callback functions
    #     self.slider_x.on_changed(self.update_x)
    #     self.slider_y.on_changed(self.update_y)
    #     self.button.on_clicked(self.reset)

    #     # Animation
    #     self.anim = animation.FuncAnimation(self.fig, self.update_plot, interval=1000 / self.sense_frequency)

    # def update_x(self, val):
    #     x_value = self.slider_x.val
    #     self.x_data = np.linspace(self.x_min, self.x_max, 100)
    #     self.y_data = np.sin(2 * np.pi * (self.x_data - x_value))
    #     self.l.set_xdata(self.x_data)
    #     self.l.set_ydata(self.y_data)
    #     self.fig.canvas.draw_idle()

    # def update_y(self, val):
    #     y_value = self.slider_y.val
    #     self.ax.set_ylim(0, y_value)
    #     self.fig.canvas.draw_idle()

    # def reset(self, event):
    #     self.slider_x.reset()
    #     self.slider_y.reset()

    # def update_plot(self, frame):
    #     self.y_data = np.roll(self.y_data, -1)
    #     self.y_data[-1] = np.sin(2 * np.pi * (self.x_data[-1] - self.slider_x.val))
    #     self.l.set_ydata(self.y_data)
    #     return self.l,

    # def show(self):
    #     plt.show()

    # def plot_input_vs_time(self, pos, timesteps, joint_name):
    #     if joint_name != 'base':
    #         # Plotting
    #         plt.plot(timesteps, pos)
    #         plt.xlabel('Timesteps')
    #         plt.ylabel('Position')
    #         plt.title('Position vs Timesteps')
    #         plt.show()
    #     else:
    #         print('Plotting Base Joint')


