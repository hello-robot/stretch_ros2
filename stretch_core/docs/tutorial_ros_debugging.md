# Debugging in ROS

This tutorial explains how to debug Stretch ROS nodes.

## VSCode ROS Extension

You can debug ROS nodes using the [ROS extension](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros).

1. Add this launch config to `.vscode/launch.json`:

```json
{
  "version": "0.2.0",
    "inputs": [
      {
        "type": "pickString",
        "id": "driverMode",
        "description": "What mode should Stretch Driver start in?",
        "options": [
          "position",
          "navigation",
          "trajectory",
          "gamepad"
        ],
        "default": "component"
      },
    ],
    "configurations": [
        {
            "name": "Driver",
            "type": "ros",
            "request": "launch",
            "target": "${workspaceFolder}/launch/stretch_driver.launch.py",
            "arguments": ["mode:=${input:driverMode}",]
        }
    ]
}
```

2. Add a `breakpoint()` statement in your code, where you want to the debugger to stop.

3. run `colcon build`

4. Run this launch config in VScode, and it should stop at the breakpoint.
