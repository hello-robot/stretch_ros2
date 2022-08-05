![](../images/banner.png)

## Overview

MoveIt 2 is a whole-body motion planning framework for mobile manipulators that allows planning pose and joint goals in environments with and without obstacles. Stretch being a mobile manipulator is uniquely well-suited to utilize the planning capabilities of MoveIt 2 in different scenarios.

## Motivation

Stretch has a kinematically simple 3 DoF arm (+2 with DexWrist) that is suitable for pick and place tasks of varied objects. Its mobile base provides it with 2 additional degrees of freedom that afford it more manipulability and also the ability to move around freely in its environment. To fully utilize these capabilities, we need a planner that can plan for both the arm and the mobile base at the same time. With MoveIt 2 and ROS 2, it is now possible to achieve this, empowering users to plan more complicated robot trajectories in difficult and uncertain environments.

## Planning with MoveIt 2 Using the MoveGroup C++ API

If you want to integrate MoveIt 2 into your planning pipeline and want greater control over its various functionalities, using the MoveGroup API is the way to go. For this tutorial we are going to use the RViz Visual Tools plugin to execute the C++ source code part by part to explore more sophisticated functionalities.

Execute the launch file again to begin the tutorial. You can follow along in the C++ script to inspect finer details. (Press Ctrl+C in the terminal to terminate) (Ensure you have enough room around the robot before running the script)

```
ros2 launch stretch_moveit_config movegroup_moveit2.launch.py
```

To execute the script and interact with the robot, all you need to do is press the Next button in the RViz Visual Tools windown at the bottom left. Follow the prompts on the terminal to run through the tutorial. While executing the script, it's also a good idea to study and understand the script that is being executed. Find it [here](https://github.com/hello-robot/stretch_ros2/blob/feature/hybrid_planning/stretch_moveit_config/src/movegroup_test.cpp).

![RVizVisualTools](https://user-images.githubusercontent.com/97639181/181854836-6f9b43cd-edf9-46e8-8ca1-1ac782d5b70b.png)


1. Let's begin by joggin the camera pan and tilt joints. For having a complete 3D representation of its environment, Stretch needs to point its head in all directions, up, down, left, right, you name it! Luckily, we have a planning group that allows you to do just that - the stretch_head planning group. Go ahead and press the Next button to jog the camera.

![Head_Group](https://user-images.githubusercontent.com/97639181/182997546-1273dcc8-e036-4eb9-93b1-5b40233501a1.gif)

2. What good is a robot that can't hold your hand on your worst days. We gave Stretch a gripper to do just that and more! Let's exercise it using the stretch_gripper planning group. All you have to do is press Next.

![Gripper_Group](https://user-images.githubusercontent.com/97639181/182997590-ad3d46df-1add-4cbc-9c5f-13a50baedc87.gif)

3. What about the good days you ask? Stretch always wants to reach out to you, no matter what. Speaking of reaching out, let's make Stretch exercise its arm for the next time you need it. Press Next.

![Arm_Group](https://user-images.githubusercontent.com/97639181/182997609-ca44b478-c645-4841-bb68-9413da208b3a.gif)

4. Stretch doesn't like sitting in a corner fretting about the future. It is the future. Stretch wants to explore and in style. What  better way to do it than by rolling around? Press Next and you'll see. That's the mobile_base planning group.

![Base_Group](https://user-images.githubusercontent.com/97639181/182997648-434c9b0f-be09-4950-bbd3-918b78ed4315.gif)

5. All that exploring does get tiring and sometimes Stretch just wants to relax and dream about its next adventure. Stretch prefers to relax with its arm down, lest someone trips over it and disturb Stretch's peaceful slumber. Press Next to see the mobile_base_arm planning group.

![Base_Arm_Group](https://user-images.githubusercontent.com/97639181/182997677-cd5de49f-aba3-4c2c-9828-a3e517b74a1e.gif)

6. Did someone say adventure? How about dodging some pesky obstacles? They're everywhere, but they don't bother Stretch a lot. It can go around them. Nothing stops Stretch! You know what to do.

![Add_Object](https://user-images.githubusercontent.com/97639181/182997701-ca93bbb0-e270-4784-b87a-741712ca2449.gif)

7. Stretch is smart, you don't need to tell it which joint goes where. Just say what you want done and it does it. How about planning a pose goal to see it in action? Press Next.

![Pose_Goal](https://user-images.githubusercontent.com/97639181/182997727-9a42f12e-9476-4cc5-a74b-71ccdecc637f.gif)

8. To wrap it up, the final act! This one is a surprise that's only a click away. Go on, do it!

![Stow](https://user-images.githubusercontent.com/97639181/182997818-7a9a104b-41a9-4cf6-8901-1a7d5ee73017.gif)

Read the comments for a breakdown of the code
