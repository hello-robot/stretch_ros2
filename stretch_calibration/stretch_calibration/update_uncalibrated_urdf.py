#!/usr/bin/env python3
import subprocess
import shlex
import sys


def main():
    print("---")
    print("Convert the current xacro file to a fresh uncalibrated URDF file.")
    print("---")

    print("ros2 run xacro xacro `ros2 pkg prefix --share stretch_description`/urdf/stretch_description.xacro > `ros2 pkg prefix --share stretch_description`/urdf/stretch_uncalibrated.urdf")

    bashCommand = "ros2 pkg prefix stretch_description"
    process = subprocess.run(shlex.split(bashCommand), capture_output=True, text=True)  
    filepath = process.stdout
    filepath = filepath[0 : len(process.stdout)-1]
    newpath = "/src/stretch_ros2/stretch_description"
    minuspath = "/install/stretch_description"
    minuslength = len(minuspath) + 1
    filepath = filepath[0 : len(process.stdout) - minuslength]
    newpath = "{0}{1}".format(filepath, newpath)
    print(newpath)

    bashCommand = "ros2 run xacro xacro {}/urdf/stretch_description.xacro".format(newpath)
    process = subprocess.run(shlex.split(bashCommand), capture_output=True, text=True)
    if(process.stderr != ''):
        print("There was an error while running the script: {}".format(process.stderr), file=sys.stderr)
        sys.exit(1)
    urdf = process.stdout

    test_filepath = "{}/urdf/stretch_uncalibrated.urdf".format(newpath)
    with open(test_filepath, "w") as open_file:
        print(urdf, file=open_file)
        open_file.close()


if __name__ == '__main__':
     main()
