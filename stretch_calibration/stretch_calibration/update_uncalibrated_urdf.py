#!/usr/bin/env python3
import subprocess
import shlex

def main():
    print("---")
    print("Convert the current xacro file to a fresh uncalibrated URDF file.")
    print("---")

    print("ros2 run xacro xacro `ros2 pkg prefix --share stretch_description`/urdf/stretch_description.xacro > `ros2 pkg prefix --share stretch_description`/urdf/stretch_uncalibrated.urdf")
    
    bashCommand = "ros2 pkg prefix --share stretch_description"
    process = subprocess.run(shlex.split(bashCommand), capture_output=True, text=True)  
    filepath = process.stdout
    filepath = filepath[0 : len(process.stdout)-1]

    bashCommand = "ros2 run xacro xacro {}/urdf/stretch_description.xacro".format(filepath)
    process = subprocess.run(shlex.split(bashCommand), capture_output=True, text=True)
    urdf = process.stdout

    test_filepath = "{}/urdf/stretch_uncalibrated.urdf".format(filepath)
    with open(test_filepath, "w") as open_file:
        print(urdf, file=open_file)
        open_file.close()

if __name__ == '__main__':
     main()
