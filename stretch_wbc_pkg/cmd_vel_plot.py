import csv
import matplotlib.pyplot as plt


class PlotCmdVelData:
    def __init__(self, base=None, arm=None, lift=None, wrist=None):
        self.base_csv_file_path = base
        self.arm_csv_file_path = arm
        self.lift_csv_file_path = lift
        self.wrist_csv_file_path = wrist
        
        self.plot = {
            "base": True if base is not None else False,
            "lift": True if lift is not None else False,
            "arm": True if arm is not None else False,
            "wrist": True if wrist is not None else False
        }
        
    def plot_main(self):
        if self.plot["base"]:
            file_path = self.base_csv_file_path
            linear = []
            angular = []
            time = []
            frame = 'base'
            
            with open(file_path, 'r') as csvfile:
                csvreader = csv.reader(csvfile)
                # Skip the header
                next(csvreader)
                for row in csvreader:
                    time.append(float(row[0]))
                    linear.append(float(row[1]))
                    angular.append(float(row[2]))
                    
            self.plot_linear_x_vs_time(time, linear, frame)
            self.plot_angular_z_vs_time(time, angular, frame)
            
        if self.plot["lift"]:
            file_path = self.lift_csv_file_path
            linear = []
            time = []
            frame = 'lift'
            
            with open(file_path, 'r') as csvfile:
                csvreader = csv.reader(csvfile)
                # Skip the header
                next(csvreader)
                for row in csvreader:
                    time.append(float(row[0]))
                    linear.append(float(row[1]))
                    
            self.plot_linear_x_vs_time(time, linear, frame)
            
        if self.plot["arm"]:
            file_path = self.arm_csv_file_path
            linear = []
            time = []
            frame = 'arm'
            
            with open(file_path, 'r') as csvfile:
                csvreader = csv.reader(csvfile)
                # Skip the header
                next(csvreader)
                for row in csvreader:
                    time.append(float(row[0]))
                    linear.append(float(row[1]))
                    
            self.plot_linear_x_vs_time(time, linear, frame)
            
        if self.plot["wrist"]:
            file_path = self.wrist_csv_file_path
            angular = []
            time = []
            frame = 'wrist'
            
            with open(file_path, 'r') as csvfile:
                csvreader = csv.reader(csvfile)
                # Skip the header
                next(csvreader)
                for row in csvreader:
                    time.append(float(row[0]))
                    angular.append(float(row[1]))
                    
            self.plot_angular_z_vs_time(time, angular, frame)
        
        self.show_plots()
                    
                
    def plot_linear_x_vs_time(self, time, linear, frame):
        plt.figure()
        plt.plot(time, linear, label='linear')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear vel')
        plt.title(f'{frame} Linear vs Time')
        plt.legend()
        plt.grid(True)
        
    def plot_angular_z_vs_time(self, time, angular, frame):
        plt.figure()
        plt.plot(time, angular, label='angular')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular vel')
        plt.title(f'{frame} Angular vs Time')
        plt.legend()
        plt.grid(True)
        
    def show_plots(self):
        plt.show()

def main():
    # File path
    base_csv_file_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/base_cmd_vel_data.csv'
    lift_csv_file_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/lift_cmd_vel_data.csv'
    arm_csv_file_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/arm_cmd_vel_data.csv'
    wrist_csv_file_path = '/home/hello-robot/ament_ws/src/stretch_ros2/stretch_wbc_pkg/plots/cmd_vel/wrist_cmd_vel_data.csv'
    p = PlotCmdVelData(base_csv_file_path, lift_csv_file_path)
    p.plot_main()
    print('done')

if __name__ == '__main__':
    main()
    

