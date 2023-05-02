## API Documentation Stretch MoveIt 2

### movegroup_moveit2
This is the API documentation for the movegroup_moveit2 node in the stretch_moveit_config package of the stretch_ros2 repository. MoveIt 2 provides a way to plan for and control control the robot joints both in task and joint space.

#### Parameters
`robot_description`:
description: the robot URDF model passed as a path or as the entire file contents
default: 'stretch_description/urdf/stretch.urdf'

`semantic_config`:
description: The robot semantic configuration with robot SRDF
default: 'stretch_moveit_config/config/stretch_description.srdf'

`allow_trajectory_execution`:
description: Whether to allow trajectory execution on actual robot
default: 'True'

`fake_execution`:
description: Whether to allow fake execution
default: 'False'

`max_safe_path_cost`:
description:
default: '1'

`jiggle_fraction`:
description:
default: '0.05'

`publish_monitored_planning_scene`:
description: Whether to publish the planning scene monitor
default: 'True'

`capabilities`:
description: The non-default MoveGroup capabilities to enable
default: ''

`disable_capabilities`:
description: The default MoveGroup capabilities to disable
default: ''

`pipeline`:
description: Specify the planning pipeline
default: 'ompl'

`debug`:
description: Whether to launch in debug mode, by defualt this is False
default='False'
choices=['True', 'False']

Parameters specific to Stretch
`db`:
description: Whether to start a database. By default, we do not start a database (it can be large)
default='False'
choices=['True', 'False']

`db_path`:
description: Allow user to specify database location
default='stretch_moveit_config/default_warehouse_mongo_db'

`use_stretch_driver`:
description: Whether to launch stretch_driver separately
default='False'
choices=['True', 'False']

`use_rviz`:
description: Whether to launch RViz for visualization
default='True'
choices=['True', 'False']

#### Published Topics

#### Subscribed Topics

#### Exposed Services

### Exposed Action Servers