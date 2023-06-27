#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <pick_place_task/pick_place_task.hpp>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_place_task");

void PickPlaceTask::Parameters::loadParameters(
    const rclcpp::Node::SharedPtr &node) {
  size_t errors = 0;
  //  errors += !rosparam_shortcuts::get(node, "arm_group_name",
  //  arm_group_name);
  errors += !rosparam_shortcuts::get(node, "hand_group_name", hand_group_name);
  errors +=
      !rosparam_shortcuts::get(node, "end_effector_name", end_effector_name);
  errors += !rosparam_shortcuts::get(node, "hand_open_pose", hand_open_pose);
  errors += !rosparam_shortcuts::get(node, "hand_close_pose", hand_close_pose);
  errors += !rosparam_shortcuts::get(node, "hand_frame", hand_frame);
  rosparam_shortcuts::shutdownIfError(errors);
}

PickPlaceTask::PickPlaceTask(const rclcpp::Node::SharedPtr &node,
                             const PickPlaceTask::Parameters &parameters) {
  using namespace moveit::task_constructor;
  RCLCPP_INFO(LOGGER, "Initializing task pipeline");
  task_ = std::make_unique<Task>(); // pick_place_task
  task_->loadRobotModel(node);

  task_->setProperty("group", parameters.mobile_base_arm_group_name);
  task_->setProperty("eef", parameters.end_effector_name);
  task_->setProperty("hand", parameters.hand_group_name);
  task_->setProperty("ik_frame", parameters.hand_frame);

  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>(node);
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();

  /** Current State **/
  Stage *current_state_ptr =
      nullptr; // Forward current_state on to grasp pose generator
  {
    auto current_state =
        std::make_unique<stages::CurrentState>("current state");
    current_state_ptr = current_state.get();
    task_->add(std::move(current_state));
  }

  /** Open Hand **/
  {
    auto stage =
        std::make_unique<stages::MoveTo>("open hand", sampling_planner);
    stage->setGroup(parameters.hand_group_name);
    stage->setGoal(parameters.hand_open_pose);
    task_->add(std::move(stage));
  }

  /** Move To Object's Pose **/
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to object pose",
        stages::Connect::GroupPlannerVector{
            {parameters.mobile_base_arm_group_name, sampling_planner}});
    stage->properties().configureInitFrom(Stage::PARENT);
    task_->add(std::move(stage));
  }

  /** Pick Object **/
  Stage *attach_object_stage =
      nullptr; // Forward attach_object_stage to place pose generator
  {
    auto grasp = std::make_unique<SerialContainer>("pick object");
    task_->properties().exposeTo(grasp->properties(),
                                 {"eef", "hand", "group", "ik_frame"});
    grasp->properties().configureInitFrom(Stage::PARENT,
                                          {"eef", "hand", "group", "ik_frame"});

    /** Approach Object **/
    {
      auto stage = std::make_unique<stages::MoveRelative>("approach object",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setIKFrame(parameters.hand_frame);
      stage->setMinMaxDistance(0.1, 0.15);
      stage->properties().set("marker_ns", "approach_object");

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "odom";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /** Generate Grasp Pose **/
    {
      // Sample grasp pose
      auto stage =
          std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose(parameters.hand_open_pose);
      stage->setObject(parameters.object_name);
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("grasp pose IK",
                                                         std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(parameters.hand_frame);
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      grasp->insert(std::move(wrapper));
    }

    /** Allow Collision (hand object) **/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow collision (hand,object)");
      stage->allowCollisions(
          parameters.object_name,
          task_->getRobotModel()
              ->getJointModelGroup(parameters.hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          true);
      grasp->insert(std::move(stage));
    }

    /** Close Hand **/
    {
      auto stage =
          std::make_unique<stages::MoveTo>("close hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name);
      stage->setGoal(parameters.hand_close_pose);
      grasp->insert(std::move(stage));
    }

    /** Attach Object **/
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("attach object");
      stage->attachObject(parameters.object_name, parameters.hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    /** Allow collision (object support) **/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "allow collision (object,support)");
      stage->allowCollisions({parameters.object_name}, "table_source", true);
      grasp->insert(std::move(stage));
    }

    /** Lift object **/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lift object",
                                                          cartesian_planner);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.01, 0.1);
      stage->setIKFrame(parameters.hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "odom";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /** Forbid collision (object support) **/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "forbid collision (object,surface)");
      stage->allowCollisions({parameters.object_name}, "table_source", false);
      grasp->insert(std::move(stage));
    }

    // Add grasp container to task
    task_->add(std::move(grasp));
  }
  /** Move to Place **/
  {
    auto stage = std::make_unique<stages::Connect>(
        "move to place",
        stages::Connect::GroupPlannerVector{
            {parameters.mobile_base_arm_group_name, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(Stage::PARENT);
    task_->add(std::move(stage));
  }

  /** Place Object **/
  {
    auto place = std::make_unique<SerialContainer>("place object");
    task_->properties().exposeTo(place->properties(), {"eef", "hand", "group"});
    place->properties().configureInitFrom(Stage::PARENT,
                                          {"eef", "hand", "group"});

    /** Lower Object **/
    {
      auto stage = std::make_unique<stages::MoveRelative>("lower object",
                                                          cartesian_planner);
      stage->properties().set("marker_ns", "lower_object");
      stage->setIKFrame(parameters.hand_frame);
      stage->properties().configureInitFrom(Stage::PARENT, {"group"});
      stage->setMinMaxDistance(.01, .1);

      // Set downward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "odom";
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }

    /** Generate Place Pose **/
    {
      // Generate Place Pose
      auto stage =
          std::make_unique<stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject(parameters.object_name);

      // Set target pose
      stage->setPose(parameters.place_pose);
      stage->setMonitoredStage(
          attach_object_stage); // Hook into attach_object_stage

      // Compute IK
      auto wrapper = std::make_unique<stages::ComputeIK>("place pose IK",
                                                         std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setIKFrame(parameters.hand_frame);
      wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(Stage::INTERFACE,
                                              {"target_pose"});
      place->insert(std::move(wrapper));
    }

    /** Open Hand **/
    {
      auto stage =
          std::make_unique<stages::MoveTo>("open hand", sampling_planner);
      stage->setGroup(parameters.hand_group_name);
      stage->setGoal(parameters.hand_open_pose);
      place->insert(std::move(stage));
    }

    /** Forbid collision (hand, object) **/
    {
      auto stage = std::make_unique<stages::ModifyPlanningScene>(
          "forbid collision (hand,object)");
      stage->allowCollisions(
          parameters.object_name,
          task_->getRobotModel()
              ->getJointModelGroup(parameters.hand_group_name)
              ->getLinkModelNamesWithCollisionGeometry(),
          false);
      place->insert(std::move(stage));
    }

    /** Detach Object **/
    {
      auto stage =
          std::make_unique<stages::ModifyPlanningScene>("detach object");
      stage->detachObject(parameters.object_name, parameters.hand_frame);
      place->insert(std::move(stage));
    }

    // Add place container to task
    task_->add(std::move(place));
  }
}

bool PickPlaceTask::plan() {
  RCLCPP_INFO(LOGGER, "Start searching for task solutions");
  task_->enableIntrospection();
  try {
    task_->plan(5);
  } catch (const moveit::task_constructor::InitStageException &e) {
    RCLCPP_ERROR_STREAM(LOGGER, "Initialization failed: " << e);
    return false;
  }
  if (task_->numSolutions() == 0) {
    RCLCPP_ERROR(LOGGER, "Planning failed");
    return false;
  }
  return true;
}

bool PickPlaceTask::execute() {
  RCLCPP_INFO(LOGGER, "Executing solution trajectory");
  moveit_msgs::msg::MoveItErrorCodes execute_result =
      task_->execute(*task_->solutions().front());

  if (execute_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(
        LOGGER, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
