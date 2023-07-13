#!/usr/bin/env python3

import subprocess
import shlex
import sys
import argparse as ap
import lsb_release

def run_cmd(cmdstr):
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("configure_wrist.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def configure_wrist(wrist_type):
    # Update XACRO and URDF in stretch_description
    bashCommand = "ros2 pkg prefix stretch_description"
    process = run_cmd(bashCommand)
    installpath = process.stdout

    addpath = "/src/stretch_ros2/stretch_description/urdf/stretch_description_{}.xacro".format(wrist_type)
    minuspath = "/install/stretch_description"
    installpath = installpath[0 : len(installpath) - len(minuspath) - 1]
    srcpath = "{0}{1}".format(installpath, addpath)
    print(srcpath)

    addpath = "/src/stretch_ros2/stretch_description/urdf/stretch_description.xacro"
    xacro_path = "{0}{1}".format(installpath, addpath)
    print(xacro_path)
    
    bashCommand = "cp {0} {1}".format(srcpath, xacro_path)
    print(bashCommand)
    run_cmd(bashCommand)

    bashCommand = "ros2 run stretch_calibration update_urdf_after_xacro_change"
    print(bashCommand)
    run_cmd(bashCommand)

    # Update SRDF in stretch_moveit_config
    bashCommand = "ros2 pkg prefix stretch_moveit_config"
    process = run_cmd(bashCommand)
    installpath = process.stdout

    if lsb_release.get_os_release()["RELEASE"] == "20.04":
        addpath = "/src/stretch_ros2/stretch_moveit_config/config/stretch_description_{}.srdf".format(wrist_type)
    else: # Humble on 22.04
        addpath = "/src/stretch_ros2/stretch_moveit2/stretch_moveit_config/config/stretch_description_{}.srdf".format(wrist_type)
    minuspath = "/install/stretch_moveit_config"
    installpath = installpath[0 : len(installpath) - len(minuspath) - 1]
    srcpath = "{0}{1}".format(installpath, addpath)
    print(srcpath)

    if lsb_release.get_os_release()["RELEASE"] == "20.04":
        addpath = "/src/stretch_ros2/stretch_moveit_config/config/stretch_description.srdf"
    else: # Humble on 22.04
        addpath = "/src/stretch_ros2/stretch_moveit2/stretch_moveit_config/config/stretch_description.srdf"
    srdf_path = "{0}{1}".format(installpath, addpath)
    print(xacro_path)

    bashCommand = "cp {0} {1}".format(srcpath, srdf_path)
    print(bashCommand)
    run_cmd(bashCommand)

def main():
    try:
        parser = ap.ArgumentParser(description='Configure end-of-arm tool for stretch ROS.')
        parser.add_argument('--standard', action='store_true', help='Configures your robot to work with a standard gripper.')
        parser.add_argument('--dex', action='store_true', help='Configures your robot to work with a dex wrist gripper.')

        args, unknown = parser.parse_known_args()
        standard = args.standard
        dex = args.dex

        if standard:
            print('Configuring a standard wrist')
            configure_wrist('standard')
        elif dex:
            print('Configuring a dex wrist')
            configure_wrist('dex')

    except KeyboardInterrupt:
        node.get_logger().info('interrupt received, so shutting down')
    
if __name__ == "__main__":
    main()
