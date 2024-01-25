#!/bin/bash
set -e

# Ensure 'stretch.urdf' exists
if [ ! -f ./stretch.urdf ]; then
    echo "ERROR: Prior to running this script make sure you have created a calibrated URDF,"
    echo "       or ensure you are in the 'stretch_ros2/stretch_description/urdf' directory."
    echo ""
    exit 1
fi

# Move previous exported_urdf to exported_urdf_previous.
if [[ -d ./exported_urdf ]]; then
    echo "Moving previous exported_urdf to exported_urdf_previous..."
    mv ./exported_urdf ./exported_urdf_previous
fi

# Create new exported URDF directories.
echo "Creating new exported URDF directories..."
mkdir -p ./exported_urdf/meshes/

# Copy the mesh files and the original calibrated URDF file to the exported URDF.
echo "Copying the meshes/URDF files to the exported URDF..."
cp ../meshes/* ./exported_urdf/meshes/
cp ./stretch.urdf ./exported_urdf/

# Replace the mesh file locations in the original URDF with local directories."
echo "Replacing the mesh ROS prefix with relative prefixes..."
OLD_NAME="package://stretch_description/"
NEW_NAME="./"
rpl -q --encoding UTF-8 -i $OLD_NAME $NEW_NAME ./exported_urdf/stretch.urdf

# Copy D435i mesh from the realsense2_description ROS package to the exported URDF.
echo "Copying D435i mesh to the exported URDF..."
cp `ros2 pkg prefix realsense2_description`/share/realsense2_description/meshes/d435.dae ./exported_urdf/meshes/
OLD_NAME="file:///home/$USER/ament_ws/install/realsense2_description/share/realsense2_description/"
NEW_NAME="./"
rpl -q --encoding UTF-8 -i $OLD_NAME $NEW_NAME ./exported_urdf/stretch.urdf

# copy controller calibration file used by stretch ROS
echo "Copying controller params to the exported URDF..."
cp `ros2 pkg prefix stretch_core`/share/stretch_core/config/controller_calibration_head.yaml ./exported_urdf/

# copy license file
echo "Copying license to exported URDF..."
cp export_urdf_license_template.md  ./exported_urdf/LICENSE.md

echo "Copying the exported URDF to the fleet directory..."
mkdir -p $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
cp -rf exported_urdf/* $HELLO_FLEET_PATH/$HELLO_FLEET_ID/exported_urdf
echo ""

echo "DONE!"
