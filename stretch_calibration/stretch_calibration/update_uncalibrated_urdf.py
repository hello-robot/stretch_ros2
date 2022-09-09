#!/usr/bin/env python3
import subprocess
import shlex
import sys


def run_cmd(cmdstr):
    process = subprocess.run(shlex.split(cmdstr), capture_output=True, text=True)
    if(process.returncode != 0):
        print("update_uncalibrated_urdf.py ERROR: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    return process

def main():
    print("---")
    print("Convert the current xacro file to a fresh uncalibrated URDF file.")
    print("---")

    print("ros2 run xacro xacro `ros2 pkg prefix stretch_description`/urdf/stretch_description.xacro > `ros2 pkg prefix stretch_description`/urdf/stretch_uncalibrated.urdf")

    bashCommand = "ros2 pkg prefix stretch_description"
    process = run_cmd(bashCommand)
    installpath = process.stdout

    # Following path manipulation is necessary because 'ros2 pkg prefix' command returns the /install path of the package whereas we want to make the changes in the /src path
    # For lack of a better comamnd in ROS 2 to directly fetch the /src path, we subtract the minuspath and add the newpath to the returned filepath
    addpath = "/src/stretch_ros2/stretch_description"
    minuspath = "/install/stretch_description"
    installpath = installpath[0 : len(installpath) - len(minuspath) - 1]
    srcpath = "{0}{1}".format(installpath, addpath)

    bashCommand = "ros2 run xacro xacro {}/urdf/stretch_description.xacro".format(srcpath)
    process = run_cmd(bashCommand)
    urdf = process.stdout

    test_filepath = "{}/urdf/stretch_uncalibrated.urdf".format(srcpath)
    with open(test_filepath, "w") as open_file:
        print(urdf, file=open_file)
        open_file.close()


if __name__ == '__main__':
     main()
