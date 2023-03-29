/*
includes the ROS and MoveIt Libraries that this package uses
*/
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

/*
Get a logger for your new node and create a namespace alias
*/
static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  // Compose an MTC task from a series of stages.
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief A getter function to get the node base interface, which will be used for the executor later.
 *
 * @return rclcpp::node_interfaces::NodeBaseInterface::SharedPtr A shared pointer to the node base interface
 */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

/*
Initialize the node with specified options.
*/
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("mtc_node", options)}
{
}

/*
This class method is used to set up the planning scene that is used in the example.
It creates a cylinder with dimensions specified by object.primitives[0].dimensions and position specified by pose.position.z and pose.position.x
*/
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1, 0.02};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.5;
  pose.position.y = -0.25;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

/*
This function interfaces with the MoveIt Task Constructor task object.
It first creates a task, which includes setting some properties and adding stages.
Next, task.init() initializes the task and task.plan(5) generates a plan, stopping after 5 successful plans are found.
The next line publishes the solution to be visualized in RViz.
Finally, task.execute() executes the plan. Execution occurs via an action server interface with the RViz plugin.
*/
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException &e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5 /*max_solutions*/))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
    return;
  }

  return;
}

mtc::Task MTCTaskNode::createTask()
{
  /*
  Creates a MoveIt Task Constructor Task object and sets some initial properties:
  (1) we set the task name to “demo_task”
  (2) load the robot model
  (3) define the names of some useful frames
  (4) set those frame names as properties of the task with task.setProperty(property_name, value)
  */
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(node_);

  const auto &arm_group_name = "panda_arm";
  const auto &hand_group_name = "hand";
  const auto &hand_frame = "panda_hand";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

/*
We add an example stage to the node:
(1) current_state_ptr is set to nullptr; this creates a pointer to a stage such that we can re-use stage information in specific scenarios.
    (This line is not used at this moment, but will be used later when more stages are added to the task).
(2) we make a current_state stage (a generator stage) and add it to our task - this starts the robot off in its current state.
(3) Now that we’ve created the CurrentState stage, we save a pointer to it in the current_state_ptr for later use.
*/

// Disable warnings for this line, as it's a variable that's set but not used in this example
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
  mtc::Stage *current_state_ptr = nullptr; // Forward current_state on to grasp pose generator
#pragma GCC diagnostic pop

  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  /*
  we need to specify a solver (Either PipelinePlanner, JointInterPolationPlanner or Cartesian path).
  Also, set some properties specific for to the Cartesian planner specifically.
  */
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScaling(1.0);
  cartesian_planner->setMaxAccelerationScaling(1.0);
  cartesian_planner->setStepSize(.01);

  /*
  Add a MoveTo stage (a propagator stage) to the task that 'opens the hand':
  Since opening the hand is a relatively simple movement, we can use the joint interpolation planner.
  This stage plans a move to the “open hand” pose, which is a named pose defined in the SRDF for the panda robot.
  */
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal("open");
  task.add(std::move(stage_open_hand));

  //////////////////////////////////////
  /*            PICK STAGES           */
  //////////////////////////////////////

  /*
  Add a MoveTo stage (a connector stage) to the task that 'moves to pick':
  This stage is initialized with a name, move_to_pick, and a GroupPlannerVector that specifies the planning group and the planner.
  We then set a timeout for the stage, set the properties for the stage, and add it to our task.
  */
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner}});
  stage_move_to_pick->setTimeout(5.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  /*
  Create a pointer to a MoveIt Task Constructor stage object, and set it to nullptr for now.
  Later, we will use this to save a stage.
  */
  mtc::Stage *attach_object_stage = nullptr; // Forward attach_object_stage to place pose generator

  /*
  creates a SerialContainer (A container that can be added to our task and can hold several substages):
  In this case, we create a serial container that will contain the stages relevant to the picking action.
  Instead of adding the stages to the task, we will add the relevant stages to the serial container.
  We use exposeTo to declare the task properties from the parent task in the new serial container, and use configureInitFrom() to initialize them.
  This allows the contained stages to access these properties.
  */
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), {"eef", "group", "ik_frame"});
    grasp->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    /*
    Create a stage to approach the object:
    This stage is a MoveRelative stage (specify a relative movement from our current position).
    MoveRelative is a propagator stage: it receives the solution from its neighbouring stages and propagates it to the next or previous stage.
    Using cartesian_planner finds a solution that involves moving the end effector in a straight line.
    We set the properties, and set the minimum and maximum distance to move.
    Now we create a Vector3Stamped message to indicate the direction we want to move - in this case, in the Z direction from the hand frame.
    Finally, we add this stage to our serial container
    */
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }

    /*
    Create a stage to generate the grasp pose:
    This is a generator stage, so it computes its results without regard to the stages before and after it.
    The first stage, CurrentState is a generator stage as well - to connect the first stage and this stage, a connecting stage must be used, which we already created above.
    This code sets the stage properties, sets the pose before grasping, the angle delta, and the monitored stage.
    Angle delta is a property of the GenerateGraspPose stage that is used to determine the number of poses to generate; when generating solutions, MoveIt Task Constructor will try to grasp the object from many different orientations, with the difference between the orientations specified by the angle delta.
    The smaller the delta, the closer together the grasp orientations will be.
    When defining the current stage, we set current_state_ptr, which is now used to forward information about the object pose and shape to the inverse kinematic solver.
    This stage won’t be directly added to the serial container like previously, as we still need to do inverse kinematics on the poses it generates.
    */
    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      stage->setPreGraspPose("open");
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr); // Hook into current state

      /*
      Before we compute inverse kinematics for the poses generated above, we first need to define the frame.
      This can be done with a PoseStamped message from geometry_msgs or:
      in this case, we define the transform using Eigen transformation matrix and the name of the relevant link.
      Here, we define the transformation matrix:
      (This is the transform from the object frame to the end-effector frame)
      */
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.1;

      /*
      Create the ComputeIK stage, and give it the name 'generate pose IK' as well as the 'generate grasp pose' stage defined above.
      Some robots have multiple inverse kinematics solutions for a given pose - we set the limit on the amount of solutions to solve for up to 8.
      We also set the minimum solution distance, which is a threshold on how different solutions must be:
      if the joint positions in a solution are too similar to a previous solution, it will be marked as invalid.
      Next, we configure some additional properties, and add the ComputeIK stage to the serial container.
      */
      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      grasp->insert(std::move(wrapper));
    }
    /*
    To pick up the object, we must allow collision between the hand and the object.
    This can be done with a ModifyPlanningScene stage.
    The allowCollisions function lets us specify which collisions to disable.
    allowCollisions can be used with a container of names, so we can use getLinkModelNamesWithCollisionGeometry to get all the names of links with collision geometry in the hand group.
    */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      grasp->insert(std::move(stage));
    }
    /*
    With collisions allowed, we now can close the hand.
    This is done with a MoveTo stage, similarly to the open hand stage from above, except moving to the close position as defined in the SRDF.
    */
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("close");
      grasp->insert(std::move(stage));
    }
    /*
    Use a ModifyPlanningScene stage again, this time to attach the object to the hand using attachObject:
    Similarly to what we did with the current_state_ptr, we get a pointer to this stage for later use when generating the place pose for the object.
    */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }
    /*
    We lift the object with a MoveRelative stage, similarly to the approach_object stage.
    */
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }
  ///////////////////////////////////////
  /*            PLACE STAGES           */
  ///////////////////////////////////////

  /*
  We move on to defining the stages for placing the object.
  We start with a Connect stage to connect the two, as we will soon be using a generator stage to generate the pose for placing the object.
  */
  {
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{{arm_group_name, sampling_planner},
                                                 {hand_group_name, sampling_planner}});
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }
  /*
  We also create a serial container for the place stages.
  This is done similarly to the pick serial container.
  The next stages will be added to the serial container rather than the task.
  */
  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    /*
    This next stage generates the poses used to place the object and compute the inverse kinematics for those poses
    - it is somewhat similar to the generate grasp pose stage from the 'pick serial container'.
    We start by creating a stage to generate the poses and inheriting the task properties.
    We specify the pose where we want to place the object with a PoseStamped message from geometry_msgs - in this case, we choose y = 0.5.
    We then pass the target pose to the stage with setPose.
    Next, we use setMonitoredStage and pass it the pointer to the attach object stage from earlier.
    This allows the stage to know how the object is attached.
    */
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "object";
      target_pose_msg.pose.position.y = 0.5;
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage); // Hook into attach_object_stage

      /*
      We then create a ComputeIK stage and pass it our GeneratePlacePose stage -
      the rest follows the same logic as above with the 'pick stages'.
      */

      // Compute IK
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    /*
    Open the hand with MoveTo stage and the joint interpolation planner.
    */
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal("open");
      place->insert(std::move(stage));
    }

    /*
    We also can re-enable collisions with the object now that we no longer need to hold it.
    This is done using allowCollisions almost exactly the same way as disabling collisions, except setting the last argument to false rather than true.
    */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object", task.getRobotModel()->getJointModelGroup(hand_group_name)->getLinkModelNamesWithCollisionGeometry(), false);
      place->insert(std::move(stage));
    }
    /*
    Detach the object using detachObject
    */
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }
    /*
    Retreat from the object using a MoveRelative stage, which is done similarly to the approach object and lift object stages.
    */
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    /*
    Finish the place serial container and add it to the task.
    */
    task.add(std::move(place));
  }

  ///////////////////////////////////////
  /*            Return Home            */
  ///////////////////////////////////////

  /*
  we use a MoveTo stage and pass it the goal pose of ready, which is a pose defined in the panda SRDF.
  */
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setGoal("ready");
    task.add(std::move(stage));
  }

  return task;
}

/*
Create a node using the class defined above, and calls the class methods to set up and execute a basic MTC task.
In this example, we do not cancel the executor once the task has finished executing to keep the node alive to inspect the solutions in RViz.
*/
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface()); 
  });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}