// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>

// MTC
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#pragma once

namespace mtc_ur_robot {
using namespace moveit::task_constructor;

// prepare a demo environment from ROS parameters under pnh
void setupDemoScene(ros::NodeHandle &pnh);

class PickPlaceTask {
 public:
  PickPlaceTask(const std::string &task_name, const ros::NodeHandle &pnh);
  ~PickPlaceTask() = default;

  bool init();

  bool plan();

  bool execute();

 private:
  /**
  * @brief Loads parameters from the config/ur_robot_config.yaml file into the
  ROS parameter server.
  */
  void loadParameters();

  /**
   * @brief Adds a 'Current State' stage to the moveit Task object in which it
   * is verified that the object is not already attached to the robot before
   * starting the motion planning.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addCurrentStateStageToTask(const std::string &object, Task &t);

  /**
   * @brief Adds an 'Open Hand' stage to the moveit Task object in which the
   * 'hand' move group is subject to a motion planning (via a sampling planner
   * approach) towards the 'hand_open_pose' configuration.
   *
   * @param[in] sampling_planner The sampling based
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addOpenHandStageToTask(
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
      Stage *&initial_state_ptr, Task &t);

  /**
   * @brief Adds a 'Move To Pick' stage to the moveit Task object in which the
   * 'arm' move group is subject to a motion planning (via a sampling planner
   * approach) towards the 'pre grasp' configuration.
   *
   * @param[in] sampling_planner The sampling planner object
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addMoveToPickStageToTask(
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner, Task &t);

  /**
   * @brief Adds a 'Pick Object' stage to the moveit Task object in which a
   * sequence of stages (i.e. 'Approach Object', 'Generate Grasp Pose', 'Allow
   * Hand Object Collission', 'Close Hand', 'Attach Object', 'Allow Object
   * Support Collission','Close Hand', 'Attach Object', 'Allow Object Support
   * Collision', 'Lift Object' and 'Forbid Object Support Collision') is grouped
   * into a sequential container that makes the robot grab and lift the object
   * of interest.
   *
   * @param[in] sampling_planner The sampling planner object
   * @param[in] cartesian_planner The cartesian planner object
   * @param[in] initial_state_ptr The initial robot state
   * @param[in,out] pick_stage_ptr The robot state after the 'Pick Object' stage
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addPickObjectStageToTask(
      const std::string &object,
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
      std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
      Stage *initial_state_ptr, Stage *&pick_stage_ptr, Task &t);

  /**
   * @brief Adds an 'Approach Object' stage to the sequential stage container
   * for the 'Pick Object' phase in which the 'arm' move group is subject to a
   * motion planning (via a cartesian planner approach) towards a location that
   * is close to the object's position.
   *
   * @param[in] sampling_planner The sampling planner object
   * @param[in] cartesian_planner The cartesian planner object
   * @param[in] initial_state_ptr The initial robot state
   * @param[in,out] pick_stage_ptr The robot state after the 'Pick Object' stage
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addApproachObjectStageToContainer(
      std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Generate Grasp Pose' stage to the sequential stage container
   * for the 'Pick Object' phase in which various poses are generated in which
   * the manipulator can potentially grab the object.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in] initial_state_ptr The initial robot state
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addGenerateGraspPoseStageToContainer(
      const std::string &object, Stage *initial_state_ptr,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds an 'Allow Hand Object Collision' stage to the sequential stage
   * container for the 'Pick Object' phase in which moveit is told that
   * collisions between the object and the 'hand' move group are allowed from
   * this point onwards.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in] initial_state_ptr The initial robot state
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addAllowHandObjectCollisionStageToContainer(
      Task &t, const std::string &object,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Close Hand' stage to the sequential stage container for the
   * 'Pick Object' phase in which the 'hand' move group is subject to a motion
   * planning (via a sampling planner approach) towards the 'hand_open_pose'
   * configuration.
   *
   * @param[in] sampling_planner The sampling planner object
   * @param[in] initial_state_ptr The initial robot state
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addCloseHandStageToContainer(
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds an 'Attach Object' stage to the sequential stage container for
   * the 'Pick Object' phase in which moveit is told that the object is attached
   * to the manipulator hand from this point onwards.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addAttachObjectStageToContainer(
      const std::string &object, std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds an 'Allow Object Support Collision' stage to the sequential
   * stage container for the 'Pick Object' phase in which moveit is told that
   * collisions between the object and its support are allowed from this point
   * onwards.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addAllowObjectSupportCollisionStageToContainer(
      const std::string &object, std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Lift Object' stage to the sequential
   * stage container for the 'Pick Object' phase in which the 'arm' move group
   * and the object together are subject to a motion planning  (via a cartesian
   * planner approach)in pure positive vertical direction to lift the object
   * from its support.
   *
   * @param[in] cartesian_planner The cartesian planner object
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addLiftObjectStageToContainer(
      std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Forbid Object Support Collision' stage to the sequential
   * stage container for the 'Pick Object' phase in which moveit is told that
   * collisions between the object and its support are no longer allowed from
   * this point onwards.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addForbidObjectSupportCollisionStageToContainer(
      const std::string &object, std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Move To Place' stage to the moveit Task object in which the
   * 'arm' move group is subject to a motion planning (via a sampling planner
   * approach) towards the 'pre place' configuration.
   *
   * @param[in] sampling_planner The sampling planner object
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addMoveToPlaceStageToTask(
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner, Task &t);

  /**
   * @brief Adds a 'Place Object' stage to the moveit Task object in which a
   * sequence of stages (i.e. 'Lower Object Stage', 'Generate Place Pose', 'Open
   * Hand', 'Forbid Arm Object Collision', 'Detach Object', 'Retreat Motion') is
   * grouped into a sequential container that makes the robot place the object
   * and retract away from it.
   *
   * @param[in] sampling_planner The sampling planner object
   * @param[in] cartesian_planner The cartesian planner object
   * @param[in,out] pick_stage_ptr The robot state after the 'Pick Object' stage
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addPlaceObjectStageToTask(
      const std::string &object,
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
      std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
      Stage *pick_stage_ptr, Task &t);

  /**
   * @brief Adds a 'Lower Object' stage to the sequential stage container for
   * the 'Place Object' phase in which the 'arm' move group and the object
   * together are subject to a motion planning  (via a cartesian planner
   * approach) in pure negative vertical direction to lower the object to its
   * support.
   *
   * @param[in] cartesian_planner The cartesian planner object
   * @param[in,out] pick_stage_ptr The robot state after the 'Pick Object' stage
   * @param[in,out] container The sequential stage container for the 'Place
   * Object' stage.
   */
  void addLowerObjectStageToContainer(
      std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Generate Place Pose' stage to the sequential stage container
   * for the 'Place Object' phase in which various poses are generated in which
   * the manipulator can potentially place the object.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] pick_stage_ptr The robot state after the 'Pick Object' stage
   * @param[in,out] container The sequential stage container for the 'Place
   * Object' stage.
   */
  void addGeneratePlacePoseStageToContainer(
      const std::string &object, Stage *pick_stage_ptr,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds an 'Open Hand' stage to the moveit Task object in which the
   * 'hand' move group is subject to a motion planning (via a sampling planner
   * approach) towards the 'hand_open_pose' configuration.
   *
   * @param[in] sampling_planner The sampling based planner object
   * @param[in,out] t The moveit task containing pick- and place stages.
   */

  /**
   * @brief Adds an 'Open Hand' stage to the sequential stage container for the
   * 'Place Object' phase in which the 'hand' move group is subject to a motion
   * planning (via a sampling planner approach) towards the 'hand_open_pose'
   * configuration.
   *
   * @param[in] sampling_planner The sampling based planner object
   * @param[in,out] container The sequential stage container for the 'Place
   * Object' stage.
   */
  void addOpenHandStageToContainer(
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Forbid Arm Object Collision' stage to the sequential
   * stage container for the 'Place Object' phase in which moveit is told that
   * collisions between the arm and the object are no longer allowed from
   * this point onwards.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addForbidArmObjectCollisionStageToContainer(
      Task &t, std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Detach Object' stage to the sequential stage container for
   * the 'Place Object' phase in which moveit is told that the object is no
   * longer attached to the manipulator hand from this point onwards.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addDetachObjectStageToContainer(
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Retreat Motion' stage to the sequential stage container for
   * the 'Place Object' phase in which the 'arm' move group is subject to a
   * motion planning (via a cartesian planner approach) towards a position away
   * from the object after placing it.
   *
   * @param[in] object The name of the object that is to be pick- and placed in
   * the task.
   * @param[in,out] container The sequential stage container for the 'Pick
   * Object' stage.
   */
  void addRetreatMotionStageToContainer(
      std::shared_ptr<solvers::CartesianPath> &cartesian_planner,
      std::unique_ptr<SerialContainer> &container);

  /**
   * @brief Adds a 'Move To Home' stage to the moveit Task object in which the
   * 'arm' move group is subject to a motion planning (via a sampling planner
   * approach) towards its home position to finalize the pick and place task.
   *
   * @param[in] sampling_planner The sampling based planner object
   * @param[in,out] t The moveit task containing pick- and place stages.
   */
  void addMoveToHomeStageToTask(
      std::shared_ptr<solvers::PipelinePlanner> &sampling_planner, Task &t);

  static constexpr char LOGNAME[]{"pick_place_task"};

  ros::NodeHandle pnh_;

  std::string task_name_;
  moveit::task_constructor::TaskPtr task_;

  // planning group properties
  std::string arm_group_name_;
  std::string eef_name_;
  std::string hand_group_name_;
  std::string hand_frame_;

  // object + surface
  std::vector<std::string> support_surfaces_;
  std::string object_reference_frame_;
  std::string surface_link_;
  std::string object_name_;
  std::string world_frame_;
  std::vector<double> object_dimensions_;

  // Predefined pose targets
  std::string hand_open_pose_;
  std::string hand_close_pose_;
  std::string arm_home_pose_;

  // Pick metrics
  Eigen::Isometry3d grasp_frame_transform_;
  double approach_object_min_dist_;
  double approach_object_max_dist_;
  double lift_object_min_dist_;
  double lift_object_max_dist_;

  // Place metrics
  geometry_msgs::Pose place_pose_;
  double place_surface_offset_;
};
}  // namespace mtc_ur_robot
