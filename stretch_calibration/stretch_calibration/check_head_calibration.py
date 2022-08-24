#!/usr/bin/env python3
import subprocess
import shlex
import os
import glob

def main():
    folder_path = "{0}/{1}/calibration_ros".format(os.getenv('HELLO_FLEET_PATH'), os.getenv('HELLO_FLEET_ID'))
    
    # Finding the latest optimization file
    print("-----------------------------------------------------------")
    print("Finding the most recent optimization results file")

    file_type = '/head_calibration_result_*.yaml'
    files = glob.glob(folder_path + file_type)
    optimization_file = max(files, key=os.path.getctime)

    print("Path to the most recent optimization results file is: ", optimization_file)

    # Finding the latest controller calibraiton file
    print("-----------------------------------------------------------")
    print("Finding the most recent controller calibration file.")

    file_type = '/controller_calibration_head_*.yaml'
    files = glob.glob(folder_path + file_type)
    controller_file = max(files, key=os.path.getctime)

    print("Path to the most recent controller calibration file is: ", controller_file)

    # Finding the latest calibrated URDF file
    print("-----------------------------------------------------------")
    print("Finding the most recent calibrated URDF file.")

    file_type = '/head_calibrated*.urdf'
    files = glob.glob(folder_path + file_type)
    urdf_file = max(files, key=os.path.getctime)

    print("Path to the most recent calibrated URDF file is: ", urdf_file)

    # Launch the visualization using these three files
    print("-----------------------------------------------------------")
    print("Launch the visualization using these three files.")
    print("ros2 launch stretch_calibration visualize_head_calibration.launch optimization_result_yaml_file:=optimization_file calibrated_controller_yaml_file:=controller_file calibrated_urdf_file:=urdf_file")

    # ros2 launch stretch_calibration check_head_calibration.launch.py optimization_result_yaml_file:=$MOSTRECENT_OPTIMIZATION_FILE calibrated_controller_yaml_file:=$MOSTRECENT_CONTROLLER_FILE calibrated_urdf_file:=$MOSTRECENT_URDF

if __name__ == '__main__':
     main()