#!/usr/bin/env python3
import subprocess
import shlex

# TODO: This script is yet to be tested
def main():
    print("Attempt to update the URDF using the most recent xacro and calibration files")
    print("This should be run after any change to the xacro files that you want to be incorporated into the calibrated UDRF. It creates a new URDF for the robot using the current xacro files and the most recent head and tool calibration files")

    bashCommand = "ros2 run stretch_calibration update_uncalibrated_urdf"
    print(bashCommand)
    subprocess.run(shlex.split(bashCommand), text=True)

    bashCommand = "ros2 launch stretch_calibration use_prior_head_calibration_to_update_urdf.launch.py"
    print(bashCommand)
    subprocess.run(shlex.split(bashCommand), text=True)

    bashCommand = "ros2 run stretch_calibration update_with_most_recent_calibration"
    print(bashCommand)
    subprocess.run(shlex.split(bashCommand), text=True)

    print("Finished with attempt to update the URDF using the most recent xacro and calibration files")


if __name__ == '__main__':
     main()