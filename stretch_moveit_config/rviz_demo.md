![](../images/banner.png)

## Overview

MoveIt 2 is a whole-body motion planning framework for mobile manipulators that allows planning pose and joint goals in environments with and without obstacles. Stretch being a mobile manipulator is uniquely well-suited to utilize the planning capabilities of MoveIt 2 in different scenarios.

## Motivation

Stretch has a kinematically simple 3 DoF arm (+2 with DexWrist) that is suitable for pick and place tasks of varied objects. Its mobile base provides it with 2 additional degrees of freedom that afford it more manipulability and also the ability to move around freely in its environment. To fully utilize these capabilities, we need a planner that can plan for both the arm and the mobile base at the same time. With MoveIt 2 and ROS 2, it is now possible to achieve this, empowering users to plan more complicated robot trajectories in difficult and uncertain environments.

Before we proceed, it's always a good idea to home the robot first by running the following script so that we have the correct joint positions being published on the /joint_states topic. This is necessary for planning trajectories on Stretch with MoveIt.

```
stretch_robot_home.py
```

### Planning with MoveIt 2 Using RViz

1. The easiest way to run MoveIt 2 on your robot is through the RViz plugin. With RViz you can plan, visualize, and also execute trajectories for various planning groups on your robot. To get started using MoveIt 2 with RViz, execute the following command in a terminal. (Press Ctrl+C in the terminal to terminate)

```
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

2. You should see Stretch visualized in RViz with joint positions exactly as they appear on the actual robot (If not, home the robot and start from step 1!). You can select a Planning Group from the drop down menu that allows you to choose a group of joints to plan for and control using MoveIt. When you select a Planning Group the joints that can be controlled are highlighted with interactive markers. Let’s go ahead and select the stretch_arm planning group.

![edited_planning_group](https://user-images.githubusercontent.com/97639181/181605746-f3b141b8-af4d-4c9e-a3e0-02ec7985c9ab.gif)

3. Now, click and drag the arrow down to slide the arm lift downwards and then use the wheel to turn the gripper inwards so that it fits squarely over the robot base. At this point if the robot base glows red in RViz, it means the robot arm is in collision with the base. You should move the lift upwards slightly until the red highlight disappears.

![edited_int_markers](https://user-images.githubusercontent.com/97639181/181605790-623fef72-ba0b-4448-9240-f9ac0cb7e671.gif)

4. Now click on the plan button to see the simulated motion of the robot in RViz

![edited_robot_stow](https://user-images.githubusercontent.com/97639181/181605820-9a5390e6-4a5b-49e6-8eae-b4aaa8e070ed.gif)

5. Click the execute button to execute the plan on the actual robot. Congratulations, you just stowed the robot arm using MoveIt! (Alternatively, if you do not want to review the simulated plan, you can click ‘Plan and Execute’ to execute the planned trajectory directly)

![edited_plan_execute](https://user-images.githubusercontent.com/97639181/181605939-12160e3a-15c7-40c8-99db-71ef643ff7a6.gif)

6. Now, let’s move Stretch’s mobile base! Select the mobile_base_arm planning_group from the drop down menu. You should see the base interactive marker appear in RViz. Use the arrow to drag the base forward or backward for about 1m. Click Plan and Execute when you are done. Voila!

![edited_move_forward](https://user-images.githubusercontent.com/97639181/181606040-c3496c59-6c6c-483d-9b9b-7538c1f8093a.gif)

The mobile_base_arm planning group also allows you to execute a coordinated base and arm motion plan. Go ahead and move the markers around to plan some fun trajectories, maybe make Stretch do a Pirouette! Similarly, the stretch_gripper and stretch_head planning groups allow opening/closing the gripper and panning/tilting the camera.

7. The interactive markers are just one way to control the joints. If you want a finer control, you can switch to the Joints tab of the plugin and use the sliders to adjust the desired end state of the joints.

![edited_joint_sliders](https://user-images.githubusercontent.com/97639181/181606099-dd17005e-2d21-4006-986a-aaa7caf59e01.gif)

8. MoveIt allows you to plan not just simple trajectories but also avoid obstacles. Let’s add an obstacle to the planning scene. Click on the Scene Objects tab and select the Box object. Define a cube of dimensions 0.1x0.1x0.1m and add it to the scene using the green + button next to it. Now, place it just in front of the mobile base using the fine controls in the Change object pose/scale buttons to the right. Click on the Publish button for MoveIt to account for the object while planning.

![edited_add_obj](https://user-images.githubusercontent.com/97639181/181606170-2aea3b25-6a95-4cfc-bbf3-064f8de50811.gif)

9. Now return back to the Planning tab and define an end state such that the Box is in between the robot start and end states. Plan and Execute!

![edited_avoid_obs](https://user-images.githubusercontent.com/97639181/181606192-e42128dc-e8a6-4fb2-b27f-869a99fc414d.gif)

With a sophisticated and fully functional perception pipeline, the planning scene can represent Stretch’s surroundings accurately and allow Stretch to manipulate and navigate in a cluttered environmen

10. Feel free to explore more sophisticated planners shipped along with MoveIt 2 in the Context tab. End!
