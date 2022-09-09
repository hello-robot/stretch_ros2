#!/usr/bin/env python3
import subprocess
import shlex
import os
import glob
import sys


def run_cmd(cmdstr):
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("revert_to_previous_calibration.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def main():
    print("Attempt to revert to the previous calibration")

    print("First, attempt to move most recent calibration files to the reversion directory")
    reversion_dir = "{0}/{1}/calibration_ros/reverted/".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))

    print("Creating reversion directory if it does not already exist")
    bashCommand = "mkdir {}".format(reversion_dir)
    print(bashCommand)
    process = run_cmd(bashCommand)

    print("-----------------------------------------------------------")
    print("Finding the most recent optimization results file")

    folder_path = "{0}/{1}/calibration_ros".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))
    file_type = '/head_calibration_result_*.yaml'
    files = glob.glob(folder_path + file_type)
    optimization_file = max(files, key=os.path.getctime)

    print("Path to the most recent optimization results file is: ", optimization_file)
    print("Moving the optimization file to the reversion directory")

    bashCommand = "mv {0} {1}".format(optimization_file, reversion_dir)
    print(bashCommand)
    process = run_cmd(bashCommand)

    print("-----------------------------------------------------------")
    print("Finding the most recent controller calibration file")

    file_type = '/controller_calibration_head_*.yaml'
    files = glob.glob(folder_path + file_type)
    controller_file = max(files, key=os.path.getctime)

    print("Path to the most recent controller calibration file is: ", controller_file)
    print("Moving the controller calibration file to the reversion directory")

    bashCommand = "mv {0} {1}".format(controller_file, reversion_dir)
    print(bashCommand)
    process = run_cmd(bashCommand)

    print("-----------------------------------------------------------")
    print("Finding the most recent calibrated URDF file")

    file_type = '/head_calibrated*.urdf'
    files = glob.glob(folder_path + file_type)
    urdf_file = max(files, key=os.path.getctime)

    print("Path to the most recent calibrated URDF file is: ", urdf_file)
    print("Moving the calibrated urdf file to the reversion directory")

    bashCommand = "mv {0} {1}".format(urdf_file, reversion_dir)
    print(bashCommand)
    process = run_cmd(bashCommand)

    print("-----------------------------------------------------------")
    print("Now, update calibration using the most recent files remaining in the calibration directory")
    print("ros2 run stretch_calibration update_with_most_recent_calibration")

    bashCommand = "ros2 run stretch_calibration update_with_most_recent_calibration"
    process = run_cmd(bashCommand)

    print("-----------------------------------------------------------")
    print("Done")


if __name__ == '__main__':
     main()