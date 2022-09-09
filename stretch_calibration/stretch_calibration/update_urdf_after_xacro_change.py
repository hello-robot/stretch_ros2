#!/usr/bin/env python3
import subprocess
import shlex
import sys


def run_cmd(cmdstr):
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("update_urdf_after_xacro_change.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def main():
    print("Attempt to update the URDF using the most recent xacro and calibration files")
    print("This should be run after any change to the xacro files that you want to be incorporated into the calibrated UDRF. It creates a new URDF for the robot using the current xacro files and the most recent head and tool calibration files")

    bashCommand = "ros2 run stretch_calibration update_uncalibrated_urdf"
    print(bashCommand)
    process = run_cmd(bashCommand)
    
    bashCommand = "ros2 launch stretch_calibration use_prior_head_calibration_to_update_urdf.launch.py"
    print(bashCommand)
    process = run_cmd(bashCommand)
    
    bashCommand = "ros2 run stretch_calibration update_with_most_recent_calibration"
    print(bashCommand)
    process = run_cmd(bashCommand)
    
    print("Finished with attempt to update the URDF using the most recent xacro and calibration files")


if __name__ == '__main__':
     main()