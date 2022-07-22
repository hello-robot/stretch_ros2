#!/bin/bash

echo ""
echo "Convert the current xacro files to a fresh uncalibrated URDF file."
echo ""

echo "ros2 run xacro xacro `ros2 pkg prefix --share stretch_description`/urdf/stretch_description.xacro > `ros2 pkg prefix --share stretch_description`/urdf/stretch_uncalibrated.urdf"

ros2 run xacro xacro `ros2 pkg prefix --share stretch_description`/urdf/stretch_description.xacro > `ros2 pkg prefix --share stretch_description`/urdf/stretch_uncalibrated.urdf
