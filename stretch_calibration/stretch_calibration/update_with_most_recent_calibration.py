#!/usr/bin/env python3
import subprocess
import shlex
import os
import glob
import sys


def run_cmd(cmdstr):
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("update_with_most_recent_calibration.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def main():
    print(os.getenv('HOME'))
    bashCommand = "source {}/ament_ws/install/setup.bash".format(os.getenv('HOME'))
    # subprocess.run(shlex.split(bashCommand), capture_output=True, text=True)

    print("Attempt to update the calibration files using the most recently performed calibration.")

    # Finding the latest optimization file
    print("-----------------------------------------------------------")
    print("Finding the most recent controller calibration file")

    folder_path = "{0}/{1}/calibration_ros".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))
    file_type = '/controller_calibration_head_*.yaml'
    files = glob.glob(folder_path + file_type)
    calibration_file = max(files, key=os.path.getctime)

    print("Found: ", calibration_file)
    print("Making it the new controller calibration file.")

    bashCommand = "ros2 pkg prefix stretch_core"
    process = run_cmd(bashCommand)
    installpath = process.stdout

    addpath = "/src/stretch_ros2/stretch_core"
    minuspath = "/install/stretch_core"
    installpath = installpath[0 : len(installpath) - len(minuspath) - 1]
    srcpath = "{0}{1}".format(installpath, addpath)
    print(srcpath)

    bashCommand = "cp {0} {1}/config/controller_calibration_head.yaml".format(calibration_file, srcpath)
    print(bashCommand)
    run_cmd(bashCommand)
    # cp $MOSTRECENT `rospack find stretch_core`/config/controller_calibration_head.yaml

    print("-----------------------------------------------------------")
    print("Find the most recent calibrated URDF file.")

    file_type = '/head_calibrated*.urdf'
    files = glob.glob(folder_path + file_type)
    calibrated_urdf_file = max(files, key=os.path.getctime)

    print("Found: ", calibrated_urdf_file)
    print("Making it the new URDF file.")

    bashCommand = "ros2 pkg prefix stretch_description"
    process = run_cmd(bashCommand)
    installpath = process.stdout

    addpath = "/src/stretch_ros2/stretch_description"
    minuspath = "/install/stretch_description"
    installpath = installpath[0 : len(installpath) - len(minuspath) - 1]
    srcpath = "{0}{1}".format(installpath, addpath)
    print(srcpath)

    bashCommand = "cp {0} {1}/urdf/stretch.urdf".format(calibrated_urdf_file, srcpath)
    print(bashCommand)
    run_cmd(bashCommand)
    # cp $MOSTRECENT `rospack find stretch_description`/urdf/stretch.urdf

    print("-----------------------------------------------------------")
    print("Finished with attempt to update the calibration files.")


if __name__ == '__main__':
    main()
