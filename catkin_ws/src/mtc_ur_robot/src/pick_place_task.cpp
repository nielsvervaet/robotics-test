/*
- constant sampling_planner and cartesian_planner arguments.
  Also; consider making them member variables that are defined in the class
constructor to reduce passing complexity.

*/

#include <pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace mtc_ur_robot {

constexpr char LOGNAME[] = "moveit_task_constructor_demo";
constexpr char PickPlaceTask::LOGNAME[];

void spawnObject(moveit::planning_interface::PlanningSceneInterface &psi,
                 const moveit_msgs::CollisionObject &object) {
  /****************************************************
   *				Spawn object                   		*
   ***************************************************/
  if (!psi.applyCollisionObject(object))
    throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable(const ros::NodeHandle &pnh, std::string table_par_name) {
  /****************************************************
   *              Create Table                  		*
   ***************************************************/

  std::string table_name, table_reference_frame;
  std::vector<double> table_dimensions;
  geometry_msgs::Pose pose;
  std::size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_par_name + std::string("_name"), table_name);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_par_name + std::string("_reference_frame"),
                                     table_reference_frame);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_par_name + std::string("_dimensions"),
                                     table_dimensions);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh, table_par_name + std::string("_pose"), pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

  moveit_msgs::CollisionObject object;
  object.id = table_name;
  object.header.frame_id = table_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = table_dimensions;
  pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
  object.primitive_poses.push_back(pose);
  return object;
}

moveit_msgs::CollisionObject createObject(const ros::NodeHandle &pnh) {
  /****************************************************
   *              Create Object                  		*
   ***************************************************/

  std::string object_name, object_reference_frame;
  std::vector<double> object_dimensions;
  geometry_msgs::Pose pose;
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame",
                                    object_reference_frame);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions",
                                    object_dimensions);
  error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
  rosparam_shortcuts::shutdownIfError(LOGNAME, error);

  moveit_msgs::CollisionObject object;
  object.id = object_name;
  object.header.frame_id = object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = object_dimensions;
  pose.position.z += 0.5 * object_dimensions[0];
  object.primitive_poses.push_back(pose);
  return object;
}

void setupDemoScene(ros::NodeHandle &pnh) {
  /****************************************************
   *              Setup Demo Scene                	*
   ***************************************************/

  // Add table and object to planning scene
  ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
  moveit::planning_interface::PlanningSceneInterface psi;
  if (pnh.param("spawn_table_1", true)) {
    spawnObject(psi, createTable(pnh, "table_1"));
  }
  if (pnh.param("spawn_table_2", true)) {
    spawnObject(psi, createTable(pnh, "table_2"));
  }

  spawnObject(psi, createObject(pnh));
}

PickPlaceTask::PickPlaceTask(const std::string &task_name,
                             const ros::NodeHandle &pnh)
    : pnh_(pnh), task_name_(task_name) {
  /****************************************************
   *               Class constructor                  *
   ***************************************************/

  loadParameters();
}

void PickPlaceTask::loadParameters() {
  /****************************************************
   *               Load Parameters                    *
   ***************************************************/

  ROS_INFO_NAMED(LOGNAME, "Loading task parameters");

  // Planning group properties
  size_t errors = 0;
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_group_name",
                                     arm_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_group_name",
                                     hand_group_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "eef_name", eef_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_frame", hand_frame_);
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh_, "world_frame", world_frame_);

  // Predefined pose targets
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh_, "arm_home_pose", arm_home_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_open_pose",
                                     hand_open_pose_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "hand_close_pose",
                                     hand_close_pose_);
  // Target object
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh_, "object_name", object_name_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_dimensions",
                                     object_dimensions_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "object_reference_frame",
                                     object_reference_frame_);
  errors +=
      !rosparam_shortcuts::get(LOGNAME, pnh_, "surface_link", surface_link_);
  support_surfaces_ = {surface_link_};

  // Pick/Place metrics
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_min_dist",
                                     approach_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "approach_object_max_dist",
                                     approach_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_min_dist",
                                     lift_object_min_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "lift_object_max_dist",
                                     lift_object_max_dist_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_surface_offset",
                                     place_surface_offset_);
  errors += !rosparam_shortcuts::get(LOGNAME, pnh_, "place_pose", place_pose_);
  rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

bool PickPlaceTask::init() {
  /****************************************************
   *               Init pick place task               *
   ***************************************************/

  ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
  const std::string object = object_name_;

  // Reset ROS introspection before constructing the new object
  task_.reset();
  task_.reset(new moveit::task_constructor::Task());

  Task &t = *task_;
  t.stages()->setName(task_name_);
  t.loadRobotModel();

  // Sampling planner
  auto sampling_planner = std::make_shared<solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);
  // sampling_planner->setPlannerId("RRTConnectkConfigDefault");

  // Cartesian planner
  auto cartesian_planner = std::make_shared<solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
  // cartesian_planner->setPlannerId("RRTConnectkConfigDefault");

  // Set task properties
  t.setProperty("group", arm_group_name_);
  t.setProperty("eef", eef_name_);
  t.setProperty("hand", hand_group_name_);
  t.setProperty("hand_grasping_frame", hand_frame_);
  t.setProperty("ik_frame", hand_frame_);

  // Initialize stages and add them to the pick and place task
  addCurrentStateStageToTask(object, t);
  Stage *initial_state_ptr = nullptr;
  addOpenHandStageToTask(sampling_planner, initial_state_ptr, t);
  addMoveToPickStageToTask(sampling_planner, t);
  Stage *pick_stage_ptr = nullptr;
  addPickObjectStageToTask(object, sampling_planner, cartesian_planner,
                           initial_state_ptr, pick_stage_ptr, t);
  addMoveToPlaceStageToTask(sampling_planner, t);
  addPlaceObjectStageToTask(object, sampling_planner, cartesian_planner,
                            pick_stage_ptr, t);
  addMoveToHomeStageToTask(sampling_planner, t);

  // prepare Task structure for planning
  try {
    t.init();
  } catch (InitStageException &e) {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
    return false;
  }

  return true;
}

void PickPlaceTask::addCurrentStateStageToTask(const std::string &object,
                                               Task &t) {
  /****************************************************
   *               Current State                      *
   ***************************************************/
  auto current_state = std::make_unique<stages::CurrentState>("current state");

  // Verify that object is not attached
  auto applicability_filter = std::make_unique<stages::PredicateFilter>(
      "applicability test", std::move(current_state));
  applicability_filter->setPredicate(
      [object](const SolutionBase &s, std::string &comment) {
        if (s.start()->scene()->getCurrentState().hasAttachedBody(object)) {
          comment = "object with id '" + object +
                    "' is already attached and cannot be picked";
          return false;
        }
        return true;
      });
  t.add(std::move(applicability_filter));
}

void PickPlaceTask::addOpenHandStageToTask(
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
    Stage *&initial_state_ptr, Task &t) {
  /****************************************************
   *               Open Hand                          *
   ***************************************************/

  // TODO: Currently not functional as the endeffector group has no controllable
  // joints (just consisting of the ee_link).

  // TODO: Just publish '1' on the /gripper_joint_position/command topic to
  // open the hand; not sure if this stage is strictly required here then..?

  auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
  stage->setGroup(hand_group_name_);
  stage->setGoal(hand_open_pose_);  // hand_open_pose_ is currently undefined.
  initial_state_ptr = stage.get();  // remember to monitor grasp pose generator
  t.add(std::move(stage));
}

void PickPlaceTask::addMoveToPickStageToTask(
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner, Task &t) {
  /****************************************************
   *               Move to Pick                       *
   ***************************************************/
  auto stage = std::make_unique<stages::Connect>(
      "move to pick",
      stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
  stage->setTimeout(5.0);
  stage->properties().configureInitFrom(Stage::PARENT);
  // initial_state_ptr = stage.get();  // remember to monitor grasp pose
  // generator
  t.add(std::move(stage));
}

void PickPlaceTask::addPickObjectStageToTask(
    const std::string &object,
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
    std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
    Stage *initial_state_ptr, Stage *&pick_stage_ptr, Task &t) {
  /****************************************************
   *               Pick Object                        *
   ***************************************************/

  // Grasp serial stage container
  auto grasp = std::make_unique<SerialContainer>("pick object");
  t.properties().exposeTo(grasp->properties(),
                          {"eef", "hand", "group", "ik_frame"});
  grasp->properties().configureInitFrom(Stage::PARENT,
                                        {"eef", "hand", "group", "ik_frame"});

  // Initialize a sequence of stages to put into the grasp serialcontainer
  addApproachObjectStageToContainer(cartesian_planner, grasp);
  addGenerateGraspPoseStageToContainer(object, initial_state_ptr, grasp);
  addAllowHandObjectCollisionStageToContainer(t, object, grasp);
  addCloseHandStageToContainer(sampling_planner, grasp);
  addAttachObjectStageToContainer(object, grasp);
  addAllowObjectSupportCollisionStageToContainer(object, grasp);
  addLiftObjectStageToContainer(cartesian_planner, grasp);
  addForbidObjectSupportCollisionStageToContainer(object, grasp);
  pick_stage_ptr = grasp.get();  // remember for monitoring place pose generator

  // Add grasp container to task
  t.add(std::move(grasp));
}

void PickPlaceTask::addApproachObjectStageToContainer(
    std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
    std::unique_ptr<SerialContainer> &container) {
  /***************************************************
  ---- *               Approach Object               *
  ***************************************************/
  auto stage = std::make_unique<stages::MoveRelative>("approach object",
                                                      cartesian_planner);
  stage->properties().set("marker_ns", "approach_object");
  stage->properties().set("link", hand_frame_);
  stage->properties().configureInitFrom(Stage::PARENT, {"group"});
  stage->setMinMaxDistance(approach_object_min_dist_,
                           approach_object_max_dist_);

  // Set hand forward direction
  geometry_msgs::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  vec.vector.x = 1;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void PickPlaceTask::addGenerateGraspPoseStageToContainer(
    const std::string &object, Stage *initial_state_ptr,
    std::unique_ptr<SerialContainer> &container) {
  /***************************************************
  ---- *               Generate Grasp Pose           *
  ***************************************************/

  // Sample grasp pose
  auto stage =
      std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
  stage->properties().configureInitFrom(Stage::PARENT);
  stage->properties().set("marker_ns", "grasp_pose");
  // stage->setPreGraspPose(hand_open_pose_);
  stage->setPreGraspPose(hand_open_pose_);
  stage->setObject(object);
  stage->setAngleDelta(M_PI / 12);
  stage->setMonitoredStage(
      initial_state_ptr);  // hook into successful initial-phase solutions

  // Compute IK
  auto wrapper =
      std::make_unique<stages::ComputeIK>("grasp pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(8);
  wrapper->setMinSolutionDistance(1.0);
  wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
  wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
  wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
  container->insert(std::move(wrapper));
}

void PickPlaceTask::addAllowHandObjectCollisionStageToContainer(
    Task &t, const std::string &object,
    std::unique_ptr<SerialContainer> &container) {
  /****************************************************
  ---- *               Allow Collision (hand object)  *
  ***************************************************/
  auto stage = std::make_unique<stages::ModifyPlanningScene>(
      "allow collision (hand,object)");
  stage->allowCollisions(object,
                         t.getRobotModel()
                             ->getJointModelGroup(hand_group_name_)
                             ->getLinkModelNamesWithCollisionGeometry(),
                         true);
  container->insert(std::move(stage));
}

void PickPlaceTask::addCloseHandStageToContainer(
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
    std::unique_ptr<SerialContainer> &container) {
  /****************************************************
  ---- *               Close Hand                     *
  ***************************************************/

  // TODO: Currently not functional as the endeffector group has no controllable
  // joints (just consisting of the ee_link).

  // TODO: Just publish '1' on the /gripper_joint_position/command topic to
  // open the hand; not sure if this stage is strictly required here then..?

  auto stage = std::make_unique<stages::MoveTo>("close hand", sampling_planner);
  stage->setGroup(hand_group_name_);
  stage->setGoal(hand_close_pose_);  // hand_close_pose_ is currently undefined.

  container->insert(std::move(stage));
}

void PickPlaceTask::addAttachObjectStageToContainer(
    const std::string &object, std::unique_ptr<SerialContainer> &container) {
  /****************************************************
   *               Attach Object                      *
   ***************************************************/
  auto stage = std::make_unique<stages::ModifyPlanningScene>("attach object");
  stage->attachObject(object, hand_frame_);
  container->insert(std::move(stage));
}

void PickPlaceTask::addAllowObjectSupportCollisionStageToContainer(
    const std::string &object, std::unique_ptr<SerialContainer> &container) {
  /****************************************************
   *               Allow collision (object support)   *
   ***************************************************/
  auto stage = std::make_unique<stages::ModifyPlanningScene>(
      "allow collision (object,support)");
  stage->allowCollisions({object}, support_surfaces_, true);
  container->insert(std::move(stage));
}

void PickPlaceTask::addLiftObjectStageToContainer(
    std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
    std::unique_ptr<SerialContainer> &container) {
  /****************************************************
   *               Lift object                        *
   ***************************************************/

  auto stage =
      std::make_unique<stages::MoveRelative>("lift object", cartesian_planner);
  stage->properties().configureInitFrom(Stage::PARENT, {"group"});
  stage->setMinMaxDistance(lift_object_min_dist_, lift_object_max_dist_);
  stage->setIKFrame(hand_frame_);
  stage->properties().set("marker_ns", "lift_object");

  // Set upward direction
  geometry_msgs::Vector3Stamped vec;
  vec.header.frame_id = world_frame_;
  vec.vector.z = 1.0;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void PickPlaceTask::addForbidObjectSupportCollisionStageToContainer(
    const std::string &object, std::unique_ptr<SerialContainer> &container) {
  /****************************************************
   *               Forbid collision (object support)  *
   ***************************************************/

  auto stage = std::make_unique<stages::ModifyPlanningScene>(
      "forbid collision (object,support)");
  stage->allowCollisions({object}, support_surfaces_, false);
  container->insert(std::move(stage));
}

void PickPlaceTask::addMoveToPlaceStageToTask(
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner, Task &t) {
  /******************************************************
   *          Move to Place                             *
   *****************************************************/
  auto stage = std::make_unique<stages::Connect>(
      "move to place",
      stages::Connect::GroupPlannerVector{{arm_group_name_, sampling_planner}});
  stage->setTimeout(5.0);
  stage->properties().configureInitFrom(Stage::PARENT);
  t.add(std::move(stage));
}

void PickPlaceTask::addPlaceObjectStageToTask(
    const std::string &object,
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
    std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
    Stage *pick_stage_ptr, Task &t) {
  /******************************************************
   *          Place Object                              *
   *****************************************************/

  auto place = std::make_unique<SerialContainer>("place object");
  t.properties().exposeTo(place->properties(), {"eef", "hand", "group"});
  place->properties().configureInitFrom(Stage::PARENT,
                                        {"eef", "hand", "group"});

  // Initialize a sequence of stages to put into the place container
  addLowerObjectStageToContainer(cartesian_planner, place);
  addGeneratePlacePoseStageToContainer(object, pick_stage_ptr, place);
  addOpenHandStageToContainer(sampling_planner, place);
  addForbidArmObjectCollisionStageToContainer(t, place);
  addDetachObjectStageToContainer(place);
  addRetreatMotionStageToContainer(cartesian_planner, place);

  // Add place container to task
  t.add(std::move(place));
}

void PickPlaceTask::addLowerObjectStageToContainer(
    std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
    std::unique_ptr<SerialContainer> &container) {
  /******************************************************
   *          Lower Object                              *
   *****************************************************/
  auto stage =
      std::make_unique<stages::MoveRelative>("lower object", cartesian_planner);
  stage->properties().set("marker_ns", "lower_object");
  stage->properties().set("link", hand_frame_);
  stage->properties().configureInitFrom(Stage::PARENT, {"group"});
  stage->setMinMaxDistance(.03, .13);

  // Set downward direction
  geometry_msgs::Vector3Stamped vec;
  vec.header.frame_id = world_frame_;
  vec.vector.z = -1.0;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void PickPlaceTask::addGeneratePlacePoseStageToContainer(
    const std::string &object, Stage *pick_stage_ptr,
    std::unique_ptr<SerialContainer> &container) {
  /****************************************************
   ---- *          Generate Place Pose                *
  *****************************************************/

  // Generate Place Pose
  auto stage =
      std::make_unique<stages::GeneratePlacePose>("generate place pose");
  stage->properties().configureInitFrom(Stage::PARENT, {"ik_frame"});
  stage->properties().set("marker_ns", "place_pose");
  stage->setObject(object);

  // Set target pose
  geometry_msgs::PoseStamped p;
  p.header.frame_id = object_reference_frame_;
  p.pose = place_pose_;
  p.pose.position.z += 0.5 * object_dimensions_[0] + place_surface_offset_;
  stage->setPose(p);
  stage->setMonitoredStage(
      pick_stage_ptr);  // hook into successful pick solutions

  // Compute IK
  auto wrapper =
      std::make_unique<stages::ComputeIK>("place pose IK", std::move(stage));
  wrapper->setMaxIKSolutions(2);
  wrapper->setIKFrame(grasp_frame_transform_, hand_frame_);
  wrapper->properties().configureInitFrom(Stage::PARENT, {"eef", "group"});
  wrapper->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
  container->insert(std::move(wrapper));
}

void PickPlaceTask::addOpenHandStageToContainer(
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
    std::unique_ptr<SerialContainer> &container) {
  /******************************************************
   *          Open Hand                                 *
   *****************************************************/

  auto stage = std::make_unique<stages::MoveTo>("open hand", sampling_planner);
  stage->setGroup(hand_group_name_);
  stage->setGoal(hand_open_pose_);
  //  TO DO: JUST PUBLISH A 'hand=1' message, stage required?
  container->insert(std::move(stage));
}

void PickPlaceTask::addForbidArmObjectCollisionStageToContainer(
    Task &t, std::unique_ptr<SerialContainer> &container) {
  /*****************************************************
   *          Forbid collision (arm, object)        	 *
   *****************************************************/

  // TODO: Why not use object, but object_name member variable here? Ideally
  // consistent in all functions
  auto stage = std::make_unique<stages::ModifyPlanningScene>(
      "forbid collision (hand,object)");
  stage->allowCollisions(
      object_name_, *t.getRobotModel()->getJointModelGroup(hand_group_name_),
      false);
  container->insert(std::move(stage));
}

void PickPlaceTask::addDetachObjectStageToContainer(
    std::unique_ptr<SerialContainer> &container) {
  /******************************************************
   *          Detach Object                             *
   *****************************************************/

  // TODO: Why not use object, but object_name member variable here? Ideally
  // consistent in all functions

  auto stage = std::make_unique<stages::ModifyPlanningScene>("detach object");
  stage->detachObject(object_name_, hand_frame_);
  container->insert(std::move(stage));
}

void PickPlaceTask::addRetreatMotionStageToContainer(
    std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
    std::unique_ptr<SerialContainer> &container) {
  /******************************************************
   *          Retreat Motion                            *
   *****************************************************/

  auto stage = std::make_unique<stages::MoveRelative>("retreat after place",
                                                      cartesian_planner);
  stage->properties().configureInitFrom(Stage::PARENT, {"group"});
  stage->setMinMaxDistance(approach_object_min_dist_,
                           approach_object_max_dist_);
  stage->setIKFrame(hand_frame_);
  stage->properties().set("marker_ns", "retreat");

  geometry_msgs::Vector3Stamped vec;
  vec.header.frame_id = hand_frame_;
  vec.vector.x = -1;
  stage->setDirection(vec);
  container->insert(std::move(stage));
}

void PickPlaceTask::addMoveToHomeStageToTask(
    std::shared_ptr<solvers::PipelinePlanner> &sampling_planner, Task &t) {
  /******************************************************
   *          Move to Home                              *
   *****************************************************/

  auto stage = std::make_unique<stages::MoveTo>("move home", sampling_planner);
  stage->properties().configureInitFrom(Stage::PARENT, {"group"});
  stage->setGoal(arm_home_pose_);
  stage->restrictDirection(stages::MoveTo::FORWARD);
  t.add(std::move(stage));
}

bool PickPlaceTask::plan() {
  /******************************************************
   *          Plan                             		  *
   *****************************************************/

  ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
  int max_solutions = pnh_.param<int>("max_solutions", 10);

  return static_cast<bool>(task_->plan(max_solutions));
}

bool PickPlaceTask::execute() {
  /******************************************************
   *          Execute                             	  *
   *****************************************************/

  ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
  moveit_msgs::MoveItErrorCodes execute_result;

  execute_result = task_->execute(*task_->solutions().front());
  // // If you want to inspect the goal message, use this instead:
  // actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>
  // execute("execute_task_solution", true); execute.waitForServer();
  // moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
  // task_->solutions().front()->fillMessage(execute_goal.solution);
  // execute.sendGoalAndWait(execute_goal);
  // execute_result = execute.getResult()->error_code;

  if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR_STREAM_NAMED(
        LOGNAME, "Task execution failed and returned: " << execute_result.val);
    return false;
  }

  return true;
}
}  // namespace mtc_ur_robot
